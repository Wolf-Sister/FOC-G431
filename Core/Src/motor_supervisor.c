#include "motor_supervisor.h"
#include "motor_port.h"
#include "motor_alignment.h"
#include "board_safety_profile.h"
#include "app_config.h"
#include "cmsis_compiler.h"
#include "encoder_bench.h"
#include "motor_axis.h"
#include "motor_profile.h"
#include <string.h>
#include <math.h>

typedef struct {
    uint8_t state, role, mode, sessionValid, acceptedValid, initialValid, armPending;
    uint8_t autoPending, calibrating, remoteCalibration, eventPending;
    uint16_t baseSeq, acceptedSeq, leaseSeq;
    uint32_t challenge, token, oldToken, nonce, revision, currentLimitMa;
    uint32_t initialReceivedMs, driverStartedMs, driverStableMs, leaseMs;
    int32_t initialIqMa;
    uint8_t driverWaiting, driverConfirmed;
    motor_event_t event;
    motor_motion_config_t motion;
    motor_axis_config_t axis;
    motor_axis_zero_window_t zeroWindow;
    uint8_t axisDirty, axisStatus, axisCoordinateValid;
    int32_t axisPositionQ;
} motor_runtime_t;
typedef struct {
    uint32_t version, epoch, receivedMs;
    uint16_t seq;
    int32_t iqMa;
} motor_candidate_t;
typedef struct {
    uint32_t epoch, version, receivedMs, driverStartedMs;
    uint16_t seq;
    uint8_t appliedValid, driverStarted, outputEnabled;
} motor_adc_report_t;

static motor_runtime_t runtime;
static motor_alignment_t alignment;
static volatile uint8_t supervisorInitialized, pendingStop;
static volatile uint32_t stopEpoch, adcCount, adcLastMs, mainLastMs, tickCount;
static volatile uint16_t faultBits, firstFaultDetail, firstFaultSeq;
static volatile uint8_t firstFaultState, firstFaultMode;
static motor_candidate_t candidates[2];
static volatile uint8_t candidateIndex;
static uint32_t candidateVersion, consumedCandidate;
static motor_command_t commands[2];
static volatile uint8_t commandIndex;
static uint32_t commandVersion, adcAppliedVersion;
static motor_adc_report_t adcReport;
static volatile uint32_t adcReportGuard;
static motor_snapshot_t sampleSlots[2];
static volatile uint32_t sampleGuard;
static volatile uint8_t sampleIndex;
static uint8_t motorSampleDivider;
static uint32_t sampleSequence;

bool MotorSupervisor_Active(void) { return supervisorInitialized != 0U; }
bool MotorSupervisor_ResetReady(void)
{
    return supervisorInitialized && !pendingStop && !runtime.armPending &&
           !runtime.calibrating && !runtime.driverWaiting && !runtime.autoPending;
}
bool MotorSupervisor_PermitsOutput(void) { return supervisorInitialized && !pendingStop && !faultBits && !EncoderBench_IsActive(); }
void MotorSupervisor_ReadFault(uint8_t *state, uint8_t *mode, uint16_t *seq, uint16_t *detail)
{
    uint32_t key = MotorPort_Lock();
    *state = firstFaultState; *mode = firstFaultMode; *seq = firstFaultSeq; *detail = firstFaultDetail;
    MotorPort_Unlock(key);
}
uint32_t MotorSupervisor_Epoch(void) { return stopEpoch; }
void MotorSupervisor_MainProgress(uint32_t now) { mainLastMs = now; }

/* Best-effort freshness, deliberately not described as hardware randomness.
 * UID/timing plus a software counter can repeat across power cycles. */
static uint32_t next_nonce(void)
{
    uint32_t value = runtime.nonce + 0x9E3779B9U;
    value ^= value << 13; value ^= value >> 17; value ^= value << 5;
    while (!value || value == runtime.oldToken || value == runtime.challenge) value++;
    runtime.nonce = value;
    return value;
}

static void publish_command(uint8_t kind, uint32_t received, uint16_t seq, int32_t iq, float voltage)
{
    uint8_t next = commandIndex ^ 1U;
    motor_command_t command = {0};
    if (++commandVersion == 0U) commandVersion++;
    command.version = commandVersion;
    command.epoch = stopEpoch;
    command.kind = kind;
    command.receivedMs = received;
    command.seq = seq;
    command.iqMa = iq;
    command.currentLimitMa = runtime.currentLimitMa;
    command.alignmentVoltage = voltage;
    command.mode = runtime.mode; command.motion = runtime.motion;
    command.role = runtime.role; command.iqDirection = MotorProfile_Selected.iqDirection;
    command.axis = runtime.axis;
    commands[next] = command;
    __DMB();
    commandIndex = next;
}

static bool read_report(motor_adc_report_t *report)
{
    uint32_t attempt;
    for (attempt = 0U; attempt < 2U; attempt++) {
        uint32_t before = adcReportGuard;
        __DMB();
        if (before & 1U) continue;
        *report = adcReport;
        __DMB();
        if (before == adcReportGuard) return true;
    }
    return false;
}

static void invalidate_session(void)
{
    runtime.oldToken = runtime.token;
    runtime.token = 0U;
    runtime.sessionValid = runtime.acceptedValid = runtime.initialValid = runtime.armPending = 0U;
    runtime.challenge = next_nonce();
    consumedCandidate = candidateVersion;
}

void MotorSupervisor_Init(uint32_t seed, uint32_t now, bool automaticCalibration)
{
    uint32_t key = MotorPort_Lock();
    memset(&runtime, 0, sizeof(runtime));
    memset(&adcReport, 0, sizeof(adcReport));
    memset(commands, 0, sizeof(commands));
    memset(candidates, 0, sizeof(candidates));
    memset(sampleSlots, 0, sizeof(sampleSlots));
    MotorAlignment_Abort(&alignment);
    pendingStop = 0U; stopEpoch = adcCount = tickCount = 0U;
    faultBits = firstFaultDetail = firstFaultSeq = 0U;
    firstFaultState = firstFaultMode = 0U;
    candidateIndex = commandIndex = sampleIndex = motorSampleDivider = 0U;
    candidateVersion = consumedCandidate = commandVersion = adcAppliedVersion = 0U;
    adcReportGuard = sampleGuard = sampleSequence = 0U;
    adcLastMs = mainLastMs = now;
    runtime.state = CAN_STATE_CALIBRATING;
    runtime.role = MotorProfile_Selected.role;
    runtime.currentLimitMa = MotorProfile_Selected.initialCurrentMa;
    runtime.motion.speedLimit = MotorProfile_Selected.speedDefaultMradS;
    runtime.motion.positionSpeed = MotorProfile_Selected.positionSpeedDefaultMradS;
    runtime.motion.acceleration = MotorProfile_Selected.accelerationDefaultMradS2;
    runtime.motion.positionMin = MotorProfile_Selected.minimumQ;
    runtime.motion.positionMax = MotorProfile_Selected.maximumQ;
    runtime.axis.direction = MotorProfile_Selected.axisDirection;
    runtime.axis.referenceRaw = MotorProfile_Selected.referenceRaw;
    runtime.axis.referenceQ = MotorProfile_Selected.referenceQ;
    runtime.axis.referencePresent = MotorProfile_Selected.referencePresent;
    runtime.axis.minimumQ = MotorProfile_Selected.minimumQ;
    runtime.axis.maximumQ = MotorProfile_Selected.maximumQ;
    runtime.axisStatus = runtime.axis.referencePresent ? MOTOR_AXIS_STATUS_STALE : MOTOR_AXIS_STATUS_NO_REFERENCE;
    runtime.nonce = seed;
    runtime.challenge = next_nonce();
    runtime.autoPending = automaticCalibration && MotorProfile_Selected.alignmentAllowed &&
                          MotorProfile_Selected.nodeId != 0U && MotorProfile_Validate() &&
                          (runtime.role != MOTOR_ROLE_JOINT ||
                           (MotorProfile_Selected.installationQualified && MotorProfile_Selected.motionQualified &&
                            MotorProfile_Selected.iqDirectionValidated && MotorAxis_ConfigValid(&runtime.axis)));
    publish_command(MOTOR_CMD_OFF, now, 0U, 0, 0.0f);
    supervisorInitialized = 1U;
    MotorPort_Unlock(key);
}

void MotorSupervisor_StopISR(void)
{
    uint32_t key = MotorPort_Lock();
    MotorPort_Disable();
    if (supervisorInitialized) { stopEpoch++; pendingStop = 1U; }
    MotorPort_Unlock(key);
}

void MotorSupervisor_FaultISR(uint16_t bits, uint16_t detail)
{
    uint32_t key = MotorPort_Lock();
    MotorPort_Disable();
    if (supervisorInitialized) {
        if (faultBits == 0U) {
            firstFaultDetail = detail;
            firstFaultState = adcReport.outputEnabled ? CAN_STATE_ENABLED : runtime.state;
            firstFaultMode = runtime.mode;
            firstFaultSeq = adcReport.appliedValid ? adcReport.seq : 0U;
        }
        faultBits |= bits;
        stopEpoch++; pendingStop = 1U;
    }
    MotorPort_Unlock(key);
}

static void queue_event(uint8_t result, uint8_t detail, uint32_t value)
{
    if (runtime.remoteCalibration) {
        runtime.event.result = result; runtime.event.detail = detail; runtime.event.value = value;
        runtime.eventPending = 1U;
    }
    runtime.remoteCalibration = 0U;
}

static void finish_stop(uint32_t now)
{
    motor_health_t health;
    MotorPort_Disable();
    MotorPort_ReadHealth(&health);
    if (runtime.calibrating) queue_event(CAN_RESULT_INTERNAL, 1U, 0U);
    if (runtime.armPending) { runtime.event.result = CAN_RESULT_INTERNAL; runtime.event.detail = 1U; runtime.event.value = 0U; runtime.eventPending = 1U; }
    runtime.calibrating = runtime.autoPending = runtime.driverWaiting = runtime.driverConfirmed = 0U;
    MotorAlignment_Abort(&alignment);
    invalidate_session();
    publish_command(MOTOR_CMD_OFF, now, 0U, 0, 0.0f);
    runtime.state = faultBits ? ((faultBits & ~(MOTOR_FAULT_BIT_COMM_TIMEOUT | MOTOR_FAULT_BIT_BUS_OFF)) ?
                                CAN_STATE_FAULT : CAN_STATE_COMM_LOST) :
                               (health.electricalCalibrated ? CAN_STATE_READY : CAN_STATE_DISABLED);
    pendingStop = 0U;
}

static bool target_isr(const uint8_t data[8], uint32_t now)
{
    motor_candidate_t candidate;
    uint16_t previous, difference;
    bool previousValid;
    if (!supervisorInitialized || pendingStop || faultBits || !runtime.sessionValid ||
        (runtime.state != CAN_STATE_READY && runtime.state != CAN_STATE_ENABLED) || runtime.calibrating ||
        data[2] != runtime.mode || data[3] != 0U) return false;
    candidate.iqMa = CAN_ReadI32(data + 4);
    if (!MotorMotion_TargetValid(&runtime.motion, runtime.mode, candidate.iqMa, runtime.currentLimitMa)) return false;
    candidate.seq = CAN_ReadU16(data);
    previousValid = runtime.acceptedValid;
    previous = runtime.acceptedSeq;
    __DMB();
    if (candidateVersion != consumedCandidate && candidates[candidateIndex].epoch == stopEpoch) {
        previousValid = true; previous = candidates[candidateIndex].seq;
    }
    difference = (uint16_t)(candidate.seq - previous);
    if ((!previousValid && candidate.seq != runtime.baseSeq) ||
        (previousValid && (difference == 0U || difference > 64U))) return false;
    candidate.epoch = stopEpoch;
    candidate.receivedMs = now;
    candidate.version = candidateVersion + 1U;
    uint8_t next = candidateIndex ^ 1U;
    candidates[next] = candidate;
    __DMB();
    candidateIndex = next;
    candidateVersion = candidate.version;
    return true;
}

bool MotorSupervisor_TargetISR(const uint8_t data[8], uint32_t now)
{
    uint32_t key = MotorPort_Lock();
    bool accepted = target_isr(data, now);
    MotorPort_Unlock(key);
    return accepted;
}

static void begin_driver(uint32_t now)
{
    runtime.driverWaiting = 1U;
    runtime.driverConfirmed = 0U;
    runtime.driverStartedMs = runtime.driverStableMs = now;
    publish_command(MOTOR_CMD_BEGIN_DRIVER, runtime.initialReceivedMs, runtime.acceptedSeq, 0, 0.0f);
}

static void begin_alignment(uint32_t now, bool remote)
{
    MotorPort_InvalidateElectrical();
    runtime.state = CAN_STATE_ALIGNING;
    runtime.calibrating = 1U;
    runtime.autoPending = 0U;
    runtime.remoteCalibration = remote;
    runtime.initialValid = runtime.armPending = 0U;
    MotorAlignment_Abort(&alignment);
    begin_driver(now);
}

void MotorSupervisor_Tick(uint32_t now)
{
    motor_adc_report_t report = {0};
    motor_health_t health;
    motor_snapshot_t sample = {0};
    if (!supervisorInitialized) return;
    tickCount++;
    if (pendingStop) finish_stop(now);
    if (faultBits) return;
    MotorPort_ReadHealth(&health);
    if (adcCount && (uint32_t)(now - adcLastMs) >= MOTOR_CONTROL_STALL_MS) {
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_CONTROL_STALLED, 10U); finish_stop(now); return;
    }
    if ((health.outputEnabled || runtime.calibrating || runtime.armPending) &&
        (uint32_t)(now - mainLastMs) >= MOTOR_MAIN_STALL_MS) {
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_CONTROL_STALLED, 0U); finish_stop(now); return;
    }
    if (EncoderBench_IsActive()) return;
    if (runtime.role == 1U && MotorSupervisor_ReadSample(&sample)) {
        MotorAxis_ZeroWindowAdd(&runtime.zeroWindow, sample.capturedMs, sample.encoderSequence,
                                sample.rawAngle, sample.valid && !sample.outputEnabled);
        if ((uint32_t)(now - sample.capturedMs) > 2U) {
            runtime.axisStatus = MOTOR_AXIS_STATUS_STALE;
            runtime.axisCoordinateValid = 0U;
        }
    }
    if (read_report(&report) && report.epoch == stopEpoch && report.outputEnabled) {
        if ((uint32_t)(now - report.receivedMs) >= MOTOR_TARGET_LEASE_MS) {
            MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_COMM_TIMEOUT, 0U); finish_stop(now); return;
        }
        runtime.state = CAN_STATE_ENABLED;
        if (runtime.armPending) {
            runtime.event.result = CAN_RESULT_OK; runtime.event.detail = 0U; runtime.event.value = report.seq;
            runtime.eventPending = 1U;
        }
        runtime.armPending = 0U;
    }
    if (runtime.armPending && (uint32_t)(now - runtime.initialReceivedMs) >= MOTOR_TARGET_LEASE_MS) {
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_COMM_TIMEOUT, 0U); finish_stop(now); return;
    }
    if (runtime.calibrating && runtime.remoteCalibration &&
        (uint32_t)(now - runtime.leaseMs) >= MOTOR_TARGET_LEASE_MS) {
        queue_event(CAN_RESULT_INTERNAL, 6U, 0U);
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_COMM_TIMEOUT, 0U); finish_stop(now); return;
    }
    if (candidateVersion != consumedCandidate) {
        motor_candidate_t candidate;
        __DMB(); candidate = candidates[candidateIndex]; consumedCandidate = candidate.version;
        if (candidate.epoch == stopEpoch && runtime.sessionValid &&
            (uint32_t)(now - candidate.receivedMs) < MOTOR_TARGET_LEASE_MS) {
            runtime.acceptedValid = 1U;
            runtime.initialValid = 1U;
            runtime.acceptedSeq = candidate.seq;
            runtime.initialReceivedMs = candidate.receivedMs;
            runtime.initialIqMa = candidate.iqMa;
            if (runtime.state == CAN_STATE_ENABLED)
                publish_command(MOTOR_CMD_TARGET, candidate.receivedMs, candidate.seq, candidate.iqMa, 0.0f);
        }
    }
    if (runtime.state == CAN_STATE_CALIBRATING && health.currentCalibrated)
        runtime.state = health.electricalCalibrated ? CAN_STATE_READY : CAN_STATE_DISABLED;
    if (runtime.autoPending && health.currentCalibrated && health.encoderValid && health.busValid)
        begin_alignment(now, false);
    if (runtime.driverWaiting) {
        if ((uint32_t)(now - runtime.driverStartedMs) >= 50U) {
            MotorSupervisor_FaultISR(1U << 5, 8U); finish_stop(now); return;
        }
        if (!health.driverEnabled || health.driverFault) runtime.driverStableMs = now;
        else if ((uint32_t)(now - runtime.driverStableMs) >= 5U) {
            runtime.driverWaiting = 0U; runtime.driverConfirmed = 1U;
            if (runtime.calibrating) {
                MotorAlignment_Start(&alignment, now);
                publish_command(MOTOR_CMD_ALIGN, now, 0U, 0, 0.0f);
            }
        }
    }
    if (runtime.calibrating && runtime.driverConfirmed) {
        int direction, polePairs;
        MotorPort_GetElectricalConfig(&direction, &polePairs);
        if (!MotorSupervisor_ReadSample(&sample)) return;
        MotorAlignment_Step(&alignment, &sample, now, direction, polePairs);
        if (alignment.phase == ALIGN_FAILED) {
            queue_event(CAN_RESULT_INTERNAL, alignment.failure, 0U);
            MotorSupervisor_FaultISR(alignment.failure == 2U ? (1U << 6) :
                                    alignment.failure == 7U ? (1U << 0) : (1U << 9), alignment.failure);
            finish_stop(now); return;
        }
        if (alignment.phase == ALIGN_DONE) {
            if (!MotorPort_CompleteAlignment(alignment.zero)) {
                if (pendingStop || faultBits) { finish_stop(now); return; }
                MotorSupervisor_FaultISR(1U << 9, 3U); finish_stop(now); return;
            }
            queue_event(CAN_RESULT_OK, 0U, (uint32_t)(alignment.zero * 1000000.0f + 0.5f));
            runtime.calibrating = runtime.driverConfirmed = 0U;
            runtime.state = CAN_STATE_READY;
            invalidate_session();
            publish_command(MOTOR_CMD_OFF, now, 0U, 0, 0.0f);
        } else publish_command(MOTOR_CMD_ALIGN, now, 0U, 0, alignment.voltage);
    }
    if (runtime.armPending && runtime.driverConfirmed)
        publish_command(MOTOR_CMD_ARM, runtime.initialReceivedMs, runtime.acceptedSeq, runtime.initialIqMa, 0.0f);
}

static uint8_t get_param(uint16_t index, uint32_t *value)
{
    switch (index) {
    case 0x0001: case 0x0205: *value = APP_CAN_NODE_ID; break;
    case 0x0002: *value = runtime.role; break;
    case 0x0003: *value = (uint32_t)runtime.axis.direction; break;
    case 0x0004: *value = 0U; break;
    case 0x0005: *value = runtime.mode; break;
    case 0x0010: *value = runtime.currentLimitMa; break;
    case 0x0011: *value = runtime.motion.speedLimit; break;
    case 0x0012: *value = (uint32_t)runtime.motion.positionMin; break;
    case 0x0013: *value = (uint32_t)runtime.motion.positionMax; break;
    case 0x0014: *value = runtime.motion.positionSpeed; break;
    case 0x0015: *value = runtime.motion.acceleration; break;
    case 0x0200: *value = (uint32_t)(MotorPort_GetElectricalZero() * 1000000.0f + 0.5f); break;
    case 0x0202: {
        uint32_t key = MotorPort_Lock();
        uint8_t coordinateValid = runtime.axisCoordinateValid;
        MotorPort_Unlock(key);
        motor_health_t health; MotorPort_ReadHealth(&health);
        *value = (health.electricalCalibrated ? 1U : 0U) |
                 (coordinateValid ? 2U : 0U); break;
    }
    case 0x0206: {
        uint32_t key = MotorPort_Lock(); *value = (uint32_t)runtime.axisPositionQ; MotorPort_Unlock(key); break;
    }
    case 0x0207: {
        uint32_t key = MotorPort_Lock(); *value = runtime.axisStatus; MotorPort_Unlock(key); break;
    }
    case 0x0210: *value = MotorProfile_Selected.currentMaxMa; break;
    case 0x0211: *value = MotorProfile_Selected.speedMaxMradS; break;
    case 0x0212: *value = MotorProfile_Selected.positionSpeedMaxMradS; break;
    case 0x0213: *value = MotorProfile_Selected.accelerationMaxMradS2; break;
    case 0x0220: *value = MotorProfile_Selected.profileId; break;
    case 0x0221: *value = MotorProfile_Selected.revision; break;
    case 0x0222: *value = MotorProfile_Selected.referenceRaw; break;
    case 0x0223: *value = (uint32_t)MotorProfile_Selected.referenceQ; break;
    case 0x0224: {
        motor_snapshot_t sample;
        uint16_t meanRaw;
        if (!MotorSupervisor_ReadSample(&sample) || !sample.valid) return CAN_RESULT_BUSY;
        if (sample.outputEnabled) return CAN_RESULT_BAD_STATE;
        if ((uint32_t)(HAL_GetTick() - sample.capturedMs) > 2U) return CAN_RESULT_STALE;
        if (!MotorAxis_ZeroWindowRead(&runtime.zeroWindow, HAL_GetTick(), &meanRaw)) return CAN_RESULT_BUSY;
        *value = ((sample.encoderSequence & 0xFFFFU) << 16) | meanRaw;
        break;
    }
    case 0x0225: *value = MotorProfile_Flags(); break;
    default: return CAN_RESULT_BAD_INDEX;
    }
    return CAN_RESULT_OK;
}

static uint8_t set_param(uint16_t index, uint32_t value)
{
    if (runtime.state == CAN_STATE_ENABLED || runtime.armPending || runtime.calibrating) return CAN_RESULT_BAD_STATE;
    if (!runtime.sessionValid) return CAN_RESULT_BAD_SESSION;
    switch (index) {
    case 0x0002:
        /* Role is a build-time hardware configuration, never a live setting. */
        return CAN_RESULT_UNSUPPORTED;
    case 0x0003: {
        (void)value;
        return CAN_RESULT_UNSUPPORTED;
    }
    case 0x0004:
        if (value > 1U) return CAN_RESULT_BAD_VALUE;
        if (value != 0U) return CAN_RESULT_UNSUPPORTED;
        break;
    case 0x0005:
        if (value > 2U) return CAN_RESULT_BAD_VALUE;
        runtime.mode = (uint8_t)value; break;
    case 0x0010:
        if (!value || value > MotorProfile_Selected.currentMaxMa || (float)value * 0.001f > CURRENT_REF_HARD_MAX_A)
            return CAN_RESULT_BAD_VALUE;
        runtime.currentLimitMa = value; break;
    case 0x0011:
        if (!value || value > MotorProfile_Selected.speedMaxMradS) return CAN_RESULT_BAD_VALUE;
        runtime.motion.speedLimit = value;
        break;
    case 0x0014:
        if (!value || value > MotorProfile_Selected.positionSpeedMaxMradS) return CAN_RESULT_BAD_VALUE;
        runtime.motion.positionSpeed = value;
        break;
    case 0x0015:
        if (!value || value > MotorProfile_Selected.accelerationMaxMradS2) return CAN_RESULT_BAD_VALUE;
        runtime.motion.acceleration = value; break;
    case 0x0012: case 0x0013: {
        if (runtime.role == MOTOR_ROLE_JOINT) return CAN_RESULT_UNSUPPORTED;
        uint8_t bytes[4]; CAN_WriteU32(bytes, value);
        int32_t signedValue = CAN_ReadI32(bytes);
        if (index == 0x0012U) runtime.motion.positionMin = signedValue;
        else runtime.motion.positionMax = signedValue;
        runtime.axis.minimumQ = runtime.motion.positionMin;
        runtime.axis.maximumQ = runtime.motion.positionMax;
        runtime.axisStatus = MotorAxis_ConfigValid(&runtime.axis) ? MOTOR_AXIS_STATUS_VALID :
                             (runtime.axis.referencePresent ? MOTOR_AXIS_STATUS_LIMITS_INVALID : MOTOR_AXIS_STATUS_NO_REFERENCE);
        runtime.axisDirty = 1U;
        break;
    }
    default: return CAN_RESULT_BAD_INDEX;
    }
    runtime.initialValid = 0U;
    consumedCandidate = candidateVersion;
    runtime.revision++;
    return CAN_RESULT_OK;
}

void MotorSupervisor_Request(const uint8_t request[8], uint32_t receivedEpoch, uint32_t receivedMs, uint32_t now, uint8_t reply[8])
{
    uint8_t result = CAN_RESULT_OK;
    uint32_t value = 0U, argument = CAN_ReadU32(request + 4);
    uint16_t index = CAN_ReadU16(request + 2);
    motor_health_t health;
    if (!supervisorInitialized) { result = CAN_RESULT_BAD_STATE; goto done; }
    MotorPort_ReadHealth(&health);
    if (CAN_ProtocolIsStop(request)) { finish_stop(now); goto done; }
    if (request[0] == CAN_OP_DISARM || request[0] == CAN_OP_HOME_ABORT) { result = CAN_RESULT_BAD_VALUE; goto done; }
    if (receivedEpoch != stopEpoch || pendingStop) { result = CAN_RESULT_BAD_SESSION; goto done; }
    if (request[0] == CAN_OP_GET_PARAM) {
        result = argument ? CAN_RESULT_BAD_VALUE : get_param(index, &value); goto done;
    }
    if (request[0] == CAN_OP_SET_MECHANICAL_ZERO || request[0] == CAN_OP_SAVE_CONFIG) {
        result = CAN_RESULT_UNSUPPORTED; goto done;
    }
    if ((uint32_t)(now - receivedMs) >= MOTOR_TARGET_LEASE_MS &&
        (request[0] == CAN_OP_CAL_KEEPALIVE || request[0] == CAN_OP_ARM || request[0] == CAN_OP_CALIBRATE)) { result = CAN_RESULT_STALE; goto done; }
    if (request[0] == CAN_OP_OPEN_SESSION) {
        if (health.outputEnabled || runtime.armPending || runtime.calibrating ||
            (runtime.state != CAN_STATE_DISABLED && runtime.state != CAN_STATE_READY &&
             runtime.state != CAN_STATE_FAULT && runtime.state != CAN_STATE_COMM_LOST)) { result = CAN_RESULT_BAD_STATE; goto done; }
        if (!argument || argument != runtime.challenge) { result = CAN_RESULT_BAD_SESSION; goto done; }
        invalidate_session();
        runtime.token = next_nonce();
        runtime.sessionValid = 1U; runtime.baseSeq = index;
        value = runtime.token; goto done;
    }
    if (request[0] == CAN_OP_CLEAR_FAULT) {
        uint16_t allowed = MOTOR_FAULT_BIT_COMM_TIMEOUT | MOTOR_FAULT_BIT_BUS_OFF;
        if (!runtime.sessionValid || argument != runtime.token) { result = CAN_RESULT_BAD_SESSION; goto done; }
        if (health.outputEnabled || runtime.calibrating || !index || (index & ~allowed)) { result = CAN_RESULT_BAD_VALUE; goto done; }
        uint32_t key = MotorPort_Lock();
        faultBits &= (uint16_t)~index;
        runtime.initialValid = runtime.acceptedValid = 0U;
        consumedCandidate = candidateVersion;
        if (!faultBits) { firstFaultDetail = firstFaultSeq = 0U; runtime.state = health.electricalCalibrated ? CAN_STATE_READY : CAN_STATE_DISABLED; }
        MotorPort_Unlock(key);
        value = faultBits; goto done;
    }
    if (faultBits) { result = CAN_RESULT_FAULT_LATCHED; goto done; }
    if (request[0] == CAN_OP_SET_PARAM || request[0] == CAN_OP_SET_IQ_LIMIT ||
        request[0] == CAN_OP_SET_MODE || request[0] == CAN_OP_SET_SOURCE) {
        if (request[0] != CAN_OP_SET_PARAM && index) { result = CAN_RESULT_BAD_VALUE; goto done; }
        uint16_t parameter = request[0] == CAN_OP_SET_IQ_LIMIT ? 0x0010U :
                             request[0] == CAN_OP_SET_MODE ? 0x0005U :
                             request[0] == CAN_OP_SET_SOURCE ? 0x0004U : index;
        result = set_param(parameter, argument); value = result ? 0U : argument; goto done;
    }
    if (request[0] == CAN_OP_CAL_KEEPALIVE) {
        if (request[1] || !runtime.remoteCalibration || !runtime.calibrating) { result = CAN_RESULT_BAD_STATE; goto done; }
        uint16_t delta = (uint16_t)(index - runtime.leaseSeq);
        if (argument != runtime.token || !runtime.sessionValid) result = CAN_RESULT_BAD_SESSION;
        else if (!delta || delta > 64U) result = CAN_RESULT_STALE;
        else { runtime.leaseSeq = index; runtime.leaseMs = receivedMs; }
        goto done;
    }
    if (request[0] == CAN_OP_ARM || request[0] == CAN_OP_CALIBRATE) {
        if (EncoderBench_IsActive()) { result = CAN_RESULT_BAD_STATE; goto done; }
        if (!runtime.sessionValid || argument != runtime.token) { result = CAN_RESULT_BAD_SESSION; goto done; }
        if (health.outputEnabled || runtime.armPending || runtime.calibrating || runtime.eventPending) { result = CAN_RESULT_BAD_STATE; goto done; }
        if (!MotorProfile_Validate() || !BOARD_SAFETY_PROFILE_VALIDATED ||
            !health.currentCalibrated || !health.busValid || !health.encoderValid) {
            result = CAN_RESULT_NOT_CONFIGURED; goto done;
        }
        if (request[0] == CAN_OP_CALIBRATE) {
            if (runtime.role == MOTOR_ROLE_JOINT &&
                (!MotorProfile_Selected.installationQualified || !MotorProfile_Selected.motionQualified ||
                 !MotorProfile_Selected.alignmentAllowed)) {
                result = CAN_RESULT_NOT_CONFIGURED; goto done;
            }
            runtime.event.opcode = request[0]; runtime.event.requestId = request[1];
            runtime.leaseSeq = index; runtime.leaseMs = receivedMs;
            begin_alignment(now, true); result = CAN_RESULT_ACCEPTED; goto done;
        }
        if (runtime.state != CAN_STATE_READY || !health.electricalCalibrated) { result = CAN_RESULT_BAD_STATE; goto done; }
        if (runtime.role == MOTOR_ROLE_JOINT &&
            (!MotorProfile_Selected.installationQualified || !MotorProfile_Selected.motionQualified ||
             !MotorProfile_Selected.iqDirectionValidated || !MotorProfile_Selected.referencePresent ||
             !MotorAxis_ConfigValid(&runtime.axis))) {
            result = CAN_RESULT_NOT_CONFIGURED; goto done;
        }
        if ((runtime.role != MOTOR_ROLE_WHEEL && runtime.role != MOTOR_ROLE_JOINT) ||
            !MotorMotion_ConfigValid(&runtime.motion, runtime.mode)) {
            result = CAN_RESULT_NOT_CONFIGURED; goto done;
        }
        if (!runtime.initialValid || index != runtime.acceptedSeq ||
            (uint32_t)(now - runtime.initialReceivedMs) > MOTOR_INITIAL_TARGET_MAX_AGE_MS) {
            result = CAN_RESULT_STALE; goto done;
        }
        if ((runtime.mode == 1U && runtime.initialIqMa != 0) ||
            (runtime.mode == 2U && !MotorPort_PositionReady(runtime.initialIqMa, &runtime.motion,
                                                             runtime.role, &runtime.axis))) {
            result = CAN_RESULT_BAD_VALUE; goto done;
        }
        runtime.armPending = 1U;
        runtime.event.opcode = request[0]; runtime.event.requestId = request[1];
        begin_driver(now);
        result = CAN_RESULT_ACCEPTED; goto done;
    }
    result = CAN_RESULT_UNSUPPORTED;
done:
    reply[0] = request[0]; reply[1] = request[1]; reply[2] = result; reply[3] = 0U;
    CAN_WriteU32(reply + 4, value);
}

void MotorSupervisor_ReadStatus(CAN_NodeStatus *status, uint32_t now)
{
    motor_health_t health;
    motor_adc_report_t report = {0};
    MotorPort_ReadHealth(&health);
    (void)read_report(&report);
    uint32_t key = MotorPort_Lock();
    status->state = faultBits ? ((faultBits & ~(MOTOR_FAULT_BIT_COMM_TIMEOUT | MOTOR_FAULT_BIT_BUS_OFF)) ?
                    CAN_STATE_FAULT : CAN_STATE_COMM_LOST) : pendingStop ?
                    (health.electricalCalibrated ? CAN_STATE_READY : CAN_STATE_DISABLED) : runtime.state;
    status->faultBits = faultBits;
    status->mode = runtime.mode; status->role = runtime.role; status->source = 0U;
    status->challenge = runtime.challenge; status->configRevision = runtime.revision;
    status->acceptedSeq = runtime.acceptedValid ? runtime.acceptedSeq : 0U;
    status->appliedSeq = report.appliedValid && report.epoch == stopEpoch ? report.seq : 0U;
    status->commandAge = UINT32_MAX;
    if (report.outputEnabled && report.appliedValid && report.epoch == stopEpoch && !pendingStop && !faultBits)
        status->commandAge = now - report.receivedMs;
    if (runtime.calibrating && runtime.remoteCalibration && !pendingStop && !faultBits) {
        status->appliedSeq = runtime.leaseSeq; status->commandAge = now - runtime.leaseMs;
    }
    status->flags = (runtime.initialValid && runtime.sessionValid && !pendingStop && !faultBits && (uint32_t)(now - runtime.initialReceivedMs) <= 10U ? 1U : 0U) |
                    (report.appliedValid && report.epoch == stopEpoch ? 2U : 0U);
    status->adcProgress = (uint8_t)(adcCount / 16U);
    status->capabilities = 1U | ((uint32_t)APP_UART_ENABLE << 2) |
                           ((uint32_t)BOARD_SAFETY_PROFILE_VALIDATED << 3) |
                           ((uint32_t)health.electricalCalibrated << 5) |
                           ((uint32_t)(runtime.axisCoordinateValid &&
                             runtime.axisStatus != MOTOR_AXIS_STATUS_STALE) << 6);
    MotorPort_Unlock(key);
}

bool MotorSupervisor_TakeEvent(motor_event_t *event)
{
    if (!runtime.eventPending) return false;
    *event = runtime.event; runtime.eventPending = 0U; return true;
}

void MotorSupervisor_OnAdcProgress(uint32_t now)
{
    if (supervisorInitialized) { adcCount++; adcLastMs = now; }
}

void MotorSupervisor_ApplyAtAdcBoundary(uint32_t now)
{
    motor_command_t command;
    bool success = true;
    if (!supervisorInitialized) return;
    __DMB(); command = commands[commandIndex];
    if (adcReport.outputEnabled && adcReport.epoch == stopEpoch &&
        (uint32_t)(now - adcReport.receivedMs) >= MOTOR_TARGET_LEASE_MS) {
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_COMM_TIMEOUT, 0U); return;
    }
    if (command.version == adcAppliedVersion || !command.version) return;
    if (command.kind == MOTOR_CMD_OFF) {
        MotorPort_Disable();
        adcReportGuard++; __DMB();
        adcReport.epoch = command.epoch;
        adcReport.outputEnabled = adcReport.appliedValid = adcReport.driverStarted = 0U;
        __DMB(); adcReportGuard++;
        adcAppliedVersion = command.version; return;
    }
    uint32_t key = MotorPort_Lock();
    if (pendingStop || faultBits || EncoderBench_IsActive() || command.epoch != stopEpoch) { MotorPort_Unlock(key); return; }
    /* The received timestamp travels with the command; apply time is no renewal. */
    if ((command.kind == MOTOR_CMD_ARM || command.kind == MOTOR_CMD_TARGET ||
         (command.kind == MOTOR_CMD_BEGIN_DRIVER && runtime.armPending)) &&
        (!BOARD_SAFETY_PROFILE_VALIDATED || (uint32_t)(now - command.receivedMs) >= MOTOR_TARGET_LEASE_MS)) {
        MotorPort_Unlock(key); MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_COMM_TIMEOUT, 0U); return;
    }
    MotorPort_Unlock(key);
    if (command.kind == MOTOR_CMD_BEGIN_DRIVER) success = MotorPort_BeginDriver();
    else if (command.kind == MOTOR_CMD_ALIGN) success = MotorPort_ApplyAlignment(command.alignmentVoltage);
    else success = MotorPort_ApplyTarget(&command, command.kind == MOTOR_CMD_ARM);
    if (success) {
        adcReportGuard++; __DMB();
        adcReport.epoch = command.epoch; adcReport.version = command.version;
        if (command.kind == MOTOR_CMD_BEGIN_DRIVER) {
            adcReport.driverStarted = 1U; adcReport.driverStartedMs = now;
            adcReport.outputEnabled = adcReport.appliedValid = 0U;
        }
        if (command.kind == MOTOR_CMD_ARM || command.kind == MOTOR_CMD_TARGET) {
            adcReport.seq = command.seq; adcReport.receivedMs = command.receivedMs;
            adcReport.appliedValid = adcReport.outputEnabled = 1U;
        }
        __DMB(); adcReportGuard++;
        adcAppliedVersion = command.version;
    }
    if (!success && !pendingStop && !faultBits) {
        uint16_t detail = (command.kind == MOTOR_CMD_ARM || command.kind == MOTOR_CMD_TARGET) ?
                          MotorPort_MotionFaultDetail() : 0U;
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_INTERNAL, detail);
    }
}

void MotorSupervisor_PublishSample(uint32_t now)
{
    if (!supervisorInitialized || ++motorSampleDivider < 16U) return;
    motorSampleDivider = 0U;
    uint8_t next = sampleIndex ^ 1U;
    motor_snapshot_t sample = {0};
    MotorPort_CaptureSample(&sample, now);
    if (runtime.role == 1U) {
        int32_t jointQ;
        runtime.axisCoordinateValid = 0U;
        runtime.axisStatus = runtime.axis.referencePresent ? MOTOR_AXIS_STATUS_STALE : MOTOR_AXIS_STATUS_NO_REFERENCE;
        if (runtime.axis.referencePresent && !MotorAxis_ConfigValid(&runtime.axis))
            runtime.axisStatus = MOTOR_AXIS_STATUS_LIMITS_INVALID;
        if (sample.valid && (uint32_t)(now - sample.capturedMs) <= 2U &&
            MotorAxis_ConfigValid(&runtime.axis)) {
            motor_axis_result_t mapped = MotorAxis_MapRaw(&runtime.axis, sample.rawAngle, &jointQ, NULL);
            if (mapped == MOTOR_AXIS_OK || mapped == MOTOR_AXIS_OUTSIDE_RANGE) {
                sample.jointPositionQ = jointQ;
                sample.jointCoordinateValid = 1U;
                runtime.axisCoordinateValid = 1U;
                runtime.axisPositionQ = jointQ;
                runtime.axisStatus = mapped == MOTOR_AXIS_OK ? MOTOR_AXIS_STATUS_VALID : MOTOR_AXIS_STATUS_OUTSIDE_RANGE;
            } else runtime.axisStatus = mapped == MOTOR_AXIS_AMBIGUOUS ? MOTOR_AXIS_STATUS_AMBIGUOUS :
                                        MOTOR_AXIS_STATUS_OUTSIDE_RANGE;
        }
    }
    sample.sequence = ++sampleSequence;
    sample.appliedSeq = adcReport.seq;
    sample.appliedValid = adcReport.appliedValid && adcReport.epoch == stopEpoch && !pendingStop && !faultBits;
    sampleGuard++; __DMB();
    sampleSlots[next] = sample;
    sampleIndex = next;
    __DMB(); sampleGuard++;
}

bool MotorSupervisor_ReadSample(motor_snapshot_t *sample)
{
    uint32_t attempt;
    for (attempt = 0U; attempt < 2U; attempt++) {
        uint32_t before = sampleGuard; __DMB();
        if (before & 1U) continue;
        *sample = sampleSlots[sampleIndex]; __DMB();
        if (before == sampleGuard) return sample->sequence != 0U;
    }
    return false;
}

bool MotorSupervisor_ReadAxisStatus(int32_t *positionQ, uint8_t *coordinateValid,
                                    uint8_t *reason, uint8_t *dirty)
{
    if (!positionQ || !coordinateValid || !reason || !dirty || !supervisorInitialized) return false;
    uint32_t key = MotorPort_Lock();
    *positionQ = runtime.axisPositionQ;
    *coordinateValid = runtime.axisCoordinateValid && runtime.axisStatus != MOTOR_AXIS_STATUS_STALE;
    *reason = runtime.axisStatus;
    *dirty = runtime.axisDirty;
    MotorPort_Unlock(key);
    return true;
}
