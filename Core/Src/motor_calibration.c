#include "motor_calibration.h"
#include "can_protocol.h"
#include "main.h"
#include "motor_port.h"
#include "motor_supervisor.h"
#include <math.h>
#include <string.h>

static cal_store_t calibrationStore;
static cal_binding_t calibrationBinding;
static cal_record_t calibrationPending;
static volatile uint8_t calibrationPhase;
static uint8_t calibrationError, calibrationLoaded, calibrationRestore, calibrationEventPending;
static uint8_t calibrationRequest[8];
static uint32_t calibrationStarted, calibrationTx, calibrationAck, calibrationEpoch;
static motor_event_t calibrationEvent;
enum
{
    CAL_TASK_IDLE,
    CAL_TASK_WAIT_REPLY,
    CAL_TASK_WAIT_TX,
    CAL_TASK_QUIESCE,
    CAL_TASK_RECOVER
};

static uint32_t calibration_zero(float zero)
{
    uint32_t value;
    if (!isfinite(zero) || zero < 0.0f || zero >= 6.28318530718f)
        return UINT32_MAX;
    value = (uint32_t)(zero * 1000000.0f + 0.5f);
    return value > CAL_ZERO_MAX ? CAL_ZERO_MAX : value;
}

void MotorCalibration_Init(void)
{
    memset(&calibrationStore, 0, sizeof(calibrationStore));
    calibrationStore.slot = -1;
    calibrationPhase = calibrationLoaded = calibrationRestore = calibrationEventPending = 0U;
    if (!MotorCalibrationPort_Init(&calibrationBinding))
    {
        calibrationError = CAL_CAPACITY;
        return;
    }
    calibrationError = CalStore_Load(&calibrationStore, &MotorCalibration_IO, &calibrationBinding);
    calibrationRestore = calibrationError == CAL_OK;
    if (calibrationStore.eccSlots)
        MotorPort_InvalidateElectrical();
}
bool MotorCalibration_Busy(void) { return calibrationPhase != 0U; }
bool MotorCalibration_CanWrite(void)
{
    return calibrationPhase == CAL_TASK_QUIESCE && MotorSupervisor_Epoch() == calibrationEpoch;
}
bool MotorCalibration_HasRestore(void) { return calibrationRestore != 0U; }
bool MotorCalibration_RecoveryRequired(void) { return calibrationStore.eccSlots != 0U; }
void MotorCalibration_CancelRestore(void) { calibrationRestore = 0U; }
bool MotorCalibration_HoldsTx(void) { return calibrationPhase == CAL_TASK_WAIT_TX; }
uint32_t MotorCalibration_Error(void) { return calibrationError; }
uint32_t MotorCalibration_Sequence(void)
{
    return calibrationStore.slot >= 0 ? calibrationStore.latest.sequence : 0U;
}
uint32_t MotorCalibration_Status(void)
{
    motor_health_t h;
    uint32_t flags = 0U;
    MotorPort_ReadHealth(&h);
    if (!calibrationStore.eccSlots && calibrationStore.slot >= 0 && calibrationStore.latest.supported && calibrationStore.latest.valid &&
        calibrationStore.latest.zero <= CAL_ZERO_MAX &&
        CalStore_BindingEqual(&calibrationStore.latest.binding, &calibrationBinding))
        flags |= CAL_STATUS_SAVED;
    if (calibrationLoaded)
        flags |= CAL_STATUS_LOADED;
    if (h.electricalCalibrated &&
        (!(flags & CAL_STATUS_SAVED) ||
         calibration_zero(MotorPort_GetElectricalZero()) != calibrationStore.latest.zero))
        flags |= CAL_STATUS_DIRTY;
    if (MotorCalibration_Busy())
        flags |= CAL_STATUS_BUSY;
    if (calibrationError != CAL_OK && calibrationError != CAL_EMPTY && calibrationError != CAL_INVALIDATED)
        flags |= CAL_STATUS_ERROR;
    if (!calibrationStore.eccSlots && calibrationStore.slot >= 0 && calibrationStore.latest.supported && !calibrationStore.latest.valid)
        flags |= CAL_STATUS_INVALIDATED;
    return flags;
}
uint8_t MotorCalibration_Request(const uint8_t request[8])
{
    uint16_t index = CAN_ReadU16(request + 2);
    float zero = MotorPort_GetElectricalZero();
    if (MotorCalibration_Busy() || calibrationEventPending)
        return CAN_RESULT_BUSY;
    if (calibrationError == CAL_CAPACITY || calibrationStore.error == CAL_IO ||
        calibrationStore.error == CAL_CONFLICT)
        return CAN_RESULT_INTERNAL;
    if (index != CAN_CAL_SAVE_CONFIRM && index != CAN_CAL_INVALIDATE_CONFIRM)
        return CAN_RESULT_BAD_VALUE;
    if (index == CAN_CAL_SAVE_CONFIRM && (!isfinite(zero) || zero < 0.0f || zero >= 6.28318530718f))
        return CAN_RESULT_BAD_VALUE;
    memset(&calibrationPending, 0, sizeof(calibrationPending));
    calibrationPending.binding = calibrationBinding;
    calibrationPending.valid = index == CAN_CAL_SAVE_CONFIRM;
    calibrationPending.zero = calibrationPending.valid ? calibration_zero(zero) : 0U;
    memcpy(calibrationRequest, request, 8);
    calibrationRestore = 0U;
    calibrationStarted = HAL_GetTick();
    calibrationTx = 0U;
    calibrationEpoch = MotorSupervisor_Epoch();
    __DMB();
    calibrationPhase = CAL_TASK_WAIT_REPLY;
    return CAN_RESULT_ACCEPTED;
}
void MotorCalibration_AcceptedTx(uint8_t id, uint32_t buffer)
{
    if (calibrationPhase == CAL_TASK_WAIT_REPLY && id == calibrationRequest[1] && buffer)
    {
        calibrationTx = buffer;
        calibrationPhase = CAL_TASK_WAIT_TX;
    }
}
static void calibration_finish(uint8_t error)
{
    uint32_t key = MotorPort_Lock();
    calibrationError = error;
    calibrationLoaded = 0U;
    MotorCalibrationPort_Flush();
    MotorSupervisor_EndStorage(HAL_GetTick());
    calibrationEvent.opcode = CAN_OP_SAVE_CONFIG;
    calibrationEvent.requestId = calibrationRequest[1];
    calibrationEvent.result = error == CAL_OK ? CAN_RESULT_OK : CAN_RESULT_INTERNAL;
    calibrationEvent.detail = error;
    calibrationEvent.value = MotorCalibration_Sequence();
    calibrationEventPending = 1U;
    calibrationPhase = CAL_TASK_IDLE;
    MotorPort_Unlock(key);
}
void MotorCalibration_Poll(void)
{
    uint32_t now = HAL_GetTick();
    motor_health_t h;
    MotorPort_ReadHealth(&h);
    if (!calibrationPhase)
    {
        if (calibrationRestore && MotorSupervisor_Active() && h.currentCalibrated && h.encoderValid &&
            h.busValid)
        {
            if (MotorSupervisor_RestoreElectrical((float)calibrationStore.latest.zero * 0.000001f))
            {
                calibrationLoaded = 1U;
                calibrationRestore = 0U;
            }
        }
        return;
    }
    if (calibrationPhase < CAL_TASK_RECOVER && MotorSupervisor_Epoch() != calibrationEpoch)
    {
        calibration_finish(CAL_CANCELLED);
        return;
    }
    if ((uint32_t)(now - calibrationStarted) >= 1000U)
    {
        calibration_finish(calibrationPhase < CAL_TASK_QUIESCE ? CAL_TX_TIMEOUT : CAL_HEALTH);
        return;
    }
    if (calibrationPhase == CAL_TASK_WAIT_TX && MotorCalibrationPort_TxConfirmed(calibrationTx))
    {
        calibrationAck = now;
        calibrationPhase = CAL_TASK_QUIESCE;
    }
    if (calibrationPhase == CAL_TASK_QUIESCE && (uint32_t)(now - calibrationAck) >= 20U)
    {
        cal_store_t nextStore;
        uint8_t result;
        uint32_t key;
        if (!h.busValid || h.driverFault || h.driverEnabled || h.outputEnabled || !h.currentCalibrated ||
            !h.encoderValid)
        {
            calibration_finish(CAL_HEALTH);
            return;
        }
        if (!MotorCalibrationPort_Quiesce())
            return;
        nextStore = calibrationStore;
        result = CalStore_Save(&nextStore, &MotorCalibration_IO, &calibrationPending);
        key = MotorPort_Lock();
        calibrationStore = nextStore;
        calibrationError = result;
        MotorPort_Unlock(key);
        if (calibrationStore.eccSlots)
            MotorPort_InvalidateElectrical();
        if (!MotorCalibrationPort_Resume())
        {
            MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_INTERNAL, 0xCAU);
            calibration_finish(CAL_HEALTH);
            return;
        }
        MotorCalibrationPort_Flush();
        if (calibrationError == CAL_OK && !calibrationPending.valid)
            MotorPort_InvalidateElectrical();
        calibrationPhase = CAL_TASK_RECOVER;
    }
    if (calibrationPhase == CAL_TASK_RECOVER)
    {
        motor_snapshot_t sample;
        MotorPort_ReadHealth(&h);
        if (h.driverFault)
        {
            calibration_finish(CAL_HEALTH);
            return;
        }
        if (h.encoderValid && h.busValid && h.currentCalibrated && MotorSupervisor_ReadSample(&sample) &&
            sample.valid && (uint32_t)(HAL_GetTick() - sample.capturedMs) <= 2U)
            calibration_finish(calibrationError);
    }
}
bool MotorCalibration_TakeEvent(motor_event_t *event)
{
    if (!calibrationEventPending)
        return false;
    *event = calibrationEvent;
    calibrationEventPending = 0U;
    return true;
}
