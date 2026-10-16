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

#define MOTOR_OUTER_LOOP_DECIMATION 32U

static motor_motion_t motion;
static uint8_t motionDivider;
volatile motor_motion_fault_snapshot_t motor_motion_fault;

uint32_t MotorPort_Lock(void) { uint32_t key = __get_PRIMASK(); __disable_irq(); return key; }
void MotorPort_Unlock(uint32_t key) { __set_PRIMASK(key); }
void MotorPort_Disable(void) { foc_disarm(); }

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

bool MotorPort_BeginDriver(void) { return foc_stage2_begin_driver(); }
bool MotorPort_ApplyIq(int32_t iqMa, uint32_t limitMa, bool first)
{
    return foc_stage2_apply_iq((float)iqMa * 0.001f, (float)limitMa * 0.001f, first);
}
bool MotorPort_PositionReady(int32_t target, const motor_motion_config_t *config)
{
    encoder_cache_t e = {0};
    if (!AS5047P_EncoderCache_Read(&e) || e.stale || !isfinite(e.velocity_rad_s) ||
        fabsf(e.velocity_rad_s) > 1.0f) return false;
    int64_t counts = MotorMotion_Counts(e.turn_count, e.raw_angle);
    return MotorMotion_InRange(config, counts) &&
           fabsf(MotorMotion_Error(counts, target)) <= MOTOR_POSITION_ARM_MAX_ERROR_RAD;
}
bool MotorPort_ApplyTarget(const motor_command_t *command, bool first)
{
    if (!MotorMotion_TargetValid(&command->motion, command->mode, command->iqMa, command->currentLimitMa)) return false;
    if (command->mode == 0U) return MotorPort_ApplyIq(command->iqMa, command->currentLimitMa, first);
    if (first) {
        if (command->mode == 2U && !MotorPort_PositionReady(command->iqMa, &command->motion)) return false;
        if (!MotorPort_ApplyIq(0, command->currentLimitMa, true)) return false;
        memset(&motion, 0, sizeof(motion));
        motor_motion_fault = (motor_motion_fault_snapshot_t){0};
        motion.config = command->motion; motion.mode = command->mode; motionDivider = 0U;
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
    if (!current_loop_enable || motor_control.mode == MOTOR_TORQUE) { motionDivider = 0U; return true; }
    if (++motionDivider < MOTOR_OUTER_LOOP_DECIMATION) return true;
    motionDivider = 0U;
    encoder_cache_t e = {0};
    bool valid = AS5047P_EncoderCache_Read(&e) && !e.stale;
    if (!valid) motion.failure = MOTION_FAILURE_ENCODER;
    else if (MotorMotion_Step(&motion, MotorMotion_Counts(e.turn_count, e.raw_angle),
                              e.velocity_rad_s, motor_config.pos_p_gain, &motor_control.set_speed)) return true;
    if (!motor_motion_fault.valid) {
        motor_motion_fault_snapshot_t snapshot = {0};
        snapshot.tick = HAL_GetTick(); snapshot.cycle = DWT->CYCCNT;
        snapshot.counts = MotorMotion_Counts(e.turn_count, e.raw_angle);
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
