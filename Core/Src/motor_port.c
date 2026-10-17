#include "motor_port.h"
#include "foc.h"
#include "as5047p.h"
#include "ina219.h"
#include "pid.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include "board_safety_profile.h"
#include "encoder_bench.h"
#include "motor_profile.h"

#define MOTOR_OUTER_LOOP_DECIMATION 32U

static motor_motion_t motion;
static uint8_t motionRole;
static motor_axis_config_t motionAxis;
static uint8_t motionDivider;
volatile motor_motion_fault_snapshot_t motor_motion_fault;

/* About 1 ms between ADC-published samples. Range also rejects out-and-back
 * movement that a velocity average alone would hide. Counts remain unwrapped. */
#define POSITION_STILL_WINDOW_MS 50U
#define POSITION_STILL_MAX_GAP_MS 2U
#define POSITION_STILL_MAX_COUNTS 24LL
#define POSITION_STILL_FAST_RAD_S 5.0f
#define POSITION_STILL_SAMPLES 64U
static struct {
    int64_t counts[POSITION_STILL_SAMPLES];
    uint32_t ticks[POSITION_STILL_SAMPLES];
    uint32_t lastSequence;
    uint8_t head, count;
} positionWindow;

uint32_t MotorPort_Lock(void) { uint32_t key = __get_PRIMASK(); __disable_irq(); return key; }
void MotorPort_Unlock(uint32_t key) { __set_PRIMASK(key); }
void MotorPort_Disable(void) { foc_disarm(); positionWindow.count = 0U; }
bool MotorPort_RestoreElectrical(float zero) { return foc_restore_electrical(zero); }

void MotorPort_ReadHealth(motor_health_t *health)
{
    encoder_cache_t encoder = {0};
    bool valid = AS5047P_EncoderCache_Read(&encoder);
    health->currentCalibrated = motor_current.Calibrated;
    health->electricalCalibrated = motor_electrical_calibrated;
    health->encoderValid = valid && !encoder.stale && AS5047P_DiagnosticsHealthy() && !EncoderBench_IsActive();
    health->busValid = ina219_bus_valid && isfinite(ina219_vbus) &&
                       ina219_vbus >= FOC_BUS_MIN_VALID_V && ina219_vbus <= FOC_BUS_MAX_VALID_V &&
                       (uint32_t)(HAL_GetTick() - ina219_bus_sample_ms) < FOC_BUS_MAX_AGE_MS;
    health->busVoltage = ina219_vbus;
    health->driverFault = HAL_GPIO_ReadPin(NFAULT_GPIO_Port, NFAULT_Pin) == GPIO_PIN_RESET;
    health->driverEnabled = foc_stage2_driver_enabled();
    health->outputEnabled = current_loop_enable;
}

bool MotorPort_BeginDriver(void)
{
    motionRole = MotorProfile_Selected.role;
    motionAxis.direction = MotorProfile_Selected.axisDirection;
    motionAxis.referenceRaw = MotorProfile_Selected.referenceRaw;
    motionAxis.referenceQ = MotorProfile_Selected.referenceQ;
    motionAxis.referencePresent = MotorProfile_Selected.referencePresent;
    motionAxis.minimumQ = MotorProfile_Selected.minimumQ;
    motionAxis.maximumQ = MotorProfile_Selected.maximumQ;
    return foc_stage2_begin_driver();
}
bool MotorPort_ApplyIq(int32_t iqMa, uint32_t limitMa, bool first)
{
    return foc_stage2_apply_iq((float)iqMa * 0.001f, (float)limitMa * 0.001f, first);
}
static bool motion_position(const encoder_cache_t *e, uint8_t role,
                            const motor_axis_config_t *axis, int64_t *counts)
{
    if (role == MOTOR_ROLE_JOINT) {
        int32_t q;
        if (MotorAxis_MapRaw(axis, e->raw_angle, &q, NULL) != MOTOR_AXIS_OK) return false;
        *counts = MotorAxis_QToCounts(q);
    } else {
        *counts = MotorMotion_Counts(e->turn_count, e->raw_angle);
    }
    return true;
}

static bool joint_inside_braking_envelope(int32_t q, float jointSpeed)
{
    float decel = (float)MotorProfile_Selected.brakingDecelerationMradS2 * 0.001f;
    float speed = fabsf(jointSpeed);
    float distanceQ;
    int64_t remainingQ;
    if (q < MotorProfile_Selected.minimumQ || q > MotorProfile_Selected.maximumQ ||
        !isfinite(jointSpeed) || decel <= 0.0f) return false;
    if (jointSpeed > 0.0f) remainingQ = (int64_t)MotorProfile_Selected.maximumQ - q;
    else if (jointSpeed < 0.0f) remainingQ = (int64_t)q - MotorProfile_Selected.minimumQ;
    else return true;
    distanceQ = (speed * (float)MotorProfile_Selected.reactionTimeMs * 10.0f +
                 (speed * speed / (2.0f * decel)) * 10000.0f +
                 (float)MotorProfile_Selected.positionMarginQ);
    return isfinite(distanceQ) && distanceQ < (float)remainingQ;
}

/* Internal-fault details 0x80..0x8A identify rejected ADC-boundary commands.
 * Keep all readiness checks; diagnostic classification must never enable output. */
enum {
    APPLY_BAD_TARGET = 0x80, APPLY_ENCODER_READ = 0x81,
    APPLY_ENCODER_STALE = 0x82, APPLY_SPEED_NONFINITE = 0x83,
    APPLY_NOT_STATIONARY = 0x84, APPLY_POSITION_MAP = 0x85,
    APPLY_POSITION_RANGE = 0x86, APPLY_POSITION_ERROR = 0x87,
    APPLY_JOINT_MARGIN = 0x88, APPLY_CURRENT_REJECTED = 0x89,
    APPLY_STILL_WINDOW = 0x8A
};

/* ADC owns updates. Main-context readers take the same IRQ lock as ADC apply. */
static void update_position_window(const encoder_cache_t *e, bool valid, uint32_t now)
{
    if (!valid || current_loop_enable || !isfinite(e->velocity_rad_s) ||
        fabsf(e->velocity_rad_s) > POSITION_STILL_FAST_RAD_S) {
        positionWindow.count = 0U; return;
    }
    if (positionWindow.count) {
        uint8_t last = (positionWindow.head + POSITION_STILL_SAMPLES - 1U) % POSITION_STILL_SAMPLES;
        if ((uint32_t)(now - positionWindow.ticks[last]) > POSITION_STILL_MAX_GAP_MS) {
            positionWindow.count = 0U; return;
        }
        if (e->update_count == positionWindow.lastSequence || now == positionWindow.ticks[last]) return;
    }
    positionWindow.lastSequence = e->update_count;
    positionWindow.counts[positionWindow.head] = MotorMotion_Counts(e->turn_count, e->raw_angle);
    positionWindow.ticks[positionWindow.head] = now;
    positionWindow.head = (positionWindow.head + 1U) % POSITION_STILL_SAMPLES;
    if (positionWindow.count < POSITION_STILL_SAMPLES) positionWindow.count++;
}

static bool position_window_ready(const encoder_cache_t *e)
{
    uint32_t now = HAL_GetTick();
    int64_t counts = MotorMotion_Counts(e->turn_count, e->raw_angle);
    int64_t minimum = counts, maximum = counts;
    bool ready = false;
    uint32_t key = MotorPort_Lock();
    for (uint8_t i = 0U; i < positionWindow.count; i++) {
        uint8_t index = (positionWindow.head + POSITION_STILL_SAMPLES - 1U - i) % POSITION_STILL_SAMPLES;
        if (i == 0U && (uint32_t)(now - positionWindow.ticks[index]) > POSITION_STILL_MAX_GAP_MS) break;
        if (positionWindow.counts[index] < minimum) minimum = positionWindow.counts[index];
        if (positionWindow.counts[index] > maximum) maximum = positionWindow.counts[index];
        if (maximum - minimum > POSITION_STILL_MAX_COUNTS) break;
        if ((uint32_t)(now - positionWindow.ticks[index]) >= POSITION_STILL_WINDOW_MS) {
            ready = true; break;
        }
    }
    MotorPort_Unlock(key);
    return ready;
}

static uint8_t position_ready_detail(int32_t target, const motor_motion_config_t *config,
                                     uint8_t role, const motor_axis_config_t *axis,
                                     motor_motion_fault_snapshot_t *snapshot)
{
    encoder_cache_t e = {0};
    int64_t counts = 0;
    if (!AS5047P_EncoderCache_Read(&e)) return APPLY_ENCODER_READ;
    if (snapshot != NULL) snapshot->rawSpeed = e.velocity_rad_s;
    if (e.stale) return APPLY_ENCODER_STALE;
    if (!isfinite(e.velocity_rad_s)) return APPLY_SPEED_NONFINITE;
    if (fabsf(e.velocity_rad_s) > POSITION_STILL_FAST_RAD_S) return APPLY_NOT_STATIONARY;
    if (!position_window_ready(&e)) return APPLY_STILL_WINDOW;
    if (!motion_position(&e, role, axis, &counts)) return APPLY_POSITION_MAP;
    if (snapshot != NULL) snapshot->counts = counts;
    if (!MotorMotion_InRange(config, counts)) return APPLY_POSITION_RANGE;
    if (!(fabsf(MotorMotion_Error(counts, target)) <= MOTOR_POSITION_ARM_MAX_ERROR_RAD))
        return APPLY_POSITION_ERROR;
    return 0U;
}

bool MotorPort_PositionReady(int32_t target, const motor_motion_config_t *config,
                             uint8_t role, const motor_axis_config_t *axis)
{
    return position_ready_detail(target, config, role, axis, NULL) == 0U;
}

static bool reject_apply(uint8_t detail, const motor_command_t *command,
                          motor_motion_fault_snapshot_t *snapshot)
{
    snapshot->tick = HAL_GetTick(); snapshot->cycle = DWT->CYCCNT;
    snapshot->target = command->iqMa; snapshot->mode = command->mode;
    snapshot->detail = detail; snapshot->valid = 1U;
    snapshot->filteredSpeed = motor_control.vel_meas;
    snapshot->speedReference = motor_control.set_speed;
    snapshot->integral = speed_loop.integral_prev;
    snapshot->iqReference = motor_control.set_torque;
    snapshot->iqMeasured = motor_control.iq_meas;
    motor_motion_fault = *snapshot;
    return false;
}
bool MotorPort_ApplyTarget(const motor_command_t *command, bool first)
{
    motor_motion_fault_snapshot_t snapshot = {0};
    if (!MotorMotion_TargetValid(&command->motion, command->mode, command->iqMa, command->currentLimitMa))
        return reject_apply(APPLY_BAD_TARGET, command, &snapshot);
    motionRole = command->role; motionAxis = command->axis;
    if (command->role == MOTOR_ROLE_JOINT && command->mode == 2U &&
        (command->iqMa < (int64_t)MotorProfile_Selected.minimumQ + MotorProfile_Selected.positionMarginQ ||
         command->iqMa > (int64_t)MotorProfile_Selected.maximumQ - MotorProfile_Selected.positionMarginQ))
        return reject_apply(APPLY_JOINT_MARGIN, command, &snapshot);
    if (command->mode == 0U) {
        int32_t iqMa = command->role == MOTOR_ROLE_JOINT ?
                       command->iqMa * command->iqDirection : command->iqMa;
        if (!MotorPort_ApplyIq(iqMa, command->currentLimitMa, first))
            return reject_apply(APPLY_CURRENT_REJECTED, command, &snapshot);
        return true;
    }
    if (first) {
        if (command->mode == 2U) {
            uint8_t detail = position_ready_detail(command->iqMa, &command->motion,
                                                  command->role, &command->axis, &snapshot);
            if (detail) return reject_apply(detail, command, &snapshot);
        }
        if (!MotorPort_ApplyIq(0, command->currentLimitMa, true))
            return reject_apply(APPLY_CURRENT_REJECTED, command, &snapshot);
        memset(&motion, 0, sizeof(motion));
        motor_motion_fault = (motor_motion_fault_snapshot_t){0};
        motion.config = command->motion; motion.mode = command->mode; motionDivider = 0U;
        motionRole = command->role; motionAxis = command->axis;
        motor_control.set_speed = 0.0f;
        motor_control.spd_needs_init = 1U;
        speed_loop.limit = (float)command->currentLimitMa * 0.001f;
        motor_control.mode = command->mode == 1U ? MOTOR_SPEED : MOTOR_POSITION;
    }
    MotorMotion_SetTarget(&motion, command->iqMa);
    return true;
}
bool MotorPort_RunOuterLoop(void)
{
    if (!current_loop_enable) { motionDivider = 0U; return true; }
    if (motor_control.mode == MOTOR_TORQUE && motionRole != MOTOR_ROLE_JOINT) {
        motionDivider = 0U;
        return true;
    }
    if (++motionDivider < MOTOR_OUTER_LOOP_DECIMATION) return true;
    motionDivider = 0U;
    encoder_cache_t e = {0};
    bool valid = AS5047P_EncoderCache_Read(&e) && !e.stale;
    int64_t counts = 0;
    float velocity = e.velocity_rad_s;
    bool positionValid = valid && motion_position(&e, motionRole, &motionAxis, &counts);
    uint8_t positionLimitViolation = 0U;
    uint8_t speedLimitViolation = 0U;
    if (motionRole == MOTOR_ROLE_JOINT) {
        velocity *= (float)motionAxis.direction;
        if (fabsf(velocity) * 1000.0f > (float)MotorProfile_Selected.speedMaxMradS) {
            speedLimitViolation = 1U;
            positionValid = false;
        }
        if (valid) {
            int32_t q;
            motor_axis_result_t mapped = MotorAxis_MapRaw(&motionAxis, e.raw_angle, &q, NULL);
            if (mapped == MOTOR_AXIS_OUTSIDE_RANGE) positionLimitViolation = 1U;
            else if (mapped != MOTOR_AXIS_OK) positionValid = false;
            else if (!joint_inside_braking_envelope(q, velocity)) positionLimitViolation = 1U;
            if (positionLimitViolation) positionValid = false;
        }
    }
    if (!positionValid) motion.failure = speedLimitViolation ? MOTION_FAILURE_OVERSPEED :
                                         positionLimitViolation ? MOTION_FAILURE_POSITION : MOTION_FAILURE_ENCODER;
    else if (motor_control.mode == MOTOR_TORQUE) return true;
    else if (MotorMotion_Step(&motion, counts, velocity,
                              motor_config.pos_p_gain, &motor_control.set_speed)) {
        if (motionRole == MOTOR_ROLE_JOINT) motor_control.set_speed *= (float)motionAxis.direction;
        return true;
    }
    if (!motor_motion_fault.valid) {
        motor_motion_fault_snapshot_t snapshot = {0};
        snapshot.tick = HAL_GetTick(); snapshot.cycle = DWT->CYCCNT;
        snapshot.counts = counts;
        snapshot.target = motion.target;
        snapshot.rawSpeed = e.velocity_rad_s; snapshot.filteredSpeed = motor_control.vel_meas;
        snapshot.speedReference = motor_control.set_speed;
        snapshot.integral = speed_loop.integral_prev;
        snapshot.iqReference = motor_control.set_torque; snapshot.iqMeasured = motor_control.iq_meas;
        snapshot.mode = motion.mode; snapshot.detail = motion.failure; snapshot.valid = 1U;
        motor_motion_fault = snapshot;
    }
    return false;
}
uint16_t MotorPort_MotionFaultDetail(void) { return motor_motion_fault.detail; }
uint16_t MotorPort_MotionFaultBits(void)
{
    switch (motor_motion_fault.detail) {
    case MOTION_FAILURE_ENCODER: return 1U << 6;
    case MOTION_FAILURE_NONFINITE: return 1U << 7;
    case MOTION_FAILURE_CONFIG: return 1U << 13;
    default: return 1U << 14;
    }
}
bool MotorPort_ApplyAlignment(float voltage) { return foc_stage2_apply_alignment(voltage); }
bool MotorPort_CompleteAlignment(float zero) { return foc_stage2_complete_alignment(zero); }
void MotorPort_InvalidateElectrical(void) { motor_electrical_calibrated = 0U; }
void MotorPort_GetElectricalConfig(int *direction, int *polePairs)
{
    *direction = motor_config.dir;
    *polePairs = motor_config.pairs;
}
float MotorPort_GetElectricalZero(void) { return motor_control.zero_elec_angle; }

void MotorPort_CaptureSample(motor_snapshot_t *sample, uint32_t now)
{
    encoder_cache_t encoder = {0};
    bool valid = AS5047P_EncoderCache_Read(&encoder);
    sample->capturedMs = now;
    sample->encoderSequence = encoder.update_count;
    sample->rawAngle = encoder.raw_angle;
    sample->turns = encoder.turn_count;
    sample->rawA = motor_current.Raw_A;
    sample->rawC = motor_current.Raw_C;
    sample->velocity = encoder.velocity_rad_s;
    sample->outputEnabled = current_loop_enable;
    sample->electricalValid = motor_electrical_calibrated;
    sample->valid = valid && !encoder.stale && AS5047P_DiagnosticsHealthy() && motor_current.Calibrated &&
                    motor_fault_reason == MOTOR_FAULT_NONE;
    update_position_window(&encoder, sample->valid, now);
    sample->maxPhaseCurrent = fmaxf(fabsf(motor_current.I_A),
                              fmaxf(fabsf(motor_current.I_B), fabsf(motor_current.I_C)));
    sample->iq = motor_control.iq_meas;
    sample->id = motor_control.id_meas;
    sample->currentLimited = current_ref_limited;
    if (!current_loop_enable && sample->valid && sample->electricalValid) {
        /* Disabled samples still carry measured dq current, with no PI/output. */
        float s, c;
        float angle = (float)(motor_config.dir * motor_config.pairs) * encoder.angle_raw -
                       motor_control.zero_elec_angle;
        float alpha = -(motor_current.I_B + motor_current.I_C);
        float beta = _1_SQRT3 * (motor_current.I_B - motor_current.I_C);
        arm_sin_cos_f32(angle * RAD_TO_DEG, &s, &c);
        sample->iq = beta * c - alpha * s;
        sample->id = alpha * c + beta * s;
    }
    if (!isfinite(sample->iq) || !isfinite(sample->id) || !isfinite(sample->velocity)) sample->valid = 0U;
}
