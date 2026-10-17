#include "can.h"
#include "can_protocol.h"
#include "fdcan.h"
#include "foc.h"
#include <string.h>
#include <math.h>
#include "motor_supervisor.h"
#include "motor_port.h"
#include "motor_axis.h"
#include "motor_calibration.h"

volatile CAN_DebugTypeDef canDebug;

#if APP_CAN_ENABLE
#define CAN_QUEUE_DEPTH 8U
#define CAN_CACHE_TIME_MS 2000U
#define CAN_TX_BUFFERS (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2)
#define CAN_FAULT_BUS_OFF (1U << 11)

typedef struct { uint8_t data[8]; uint32_t epoch, receivedMs; } CAN_Packet;
typedef struct {
    uint8_t valid, request[8], reply[8];
    uint32_t created, epoch;
} CAN_CachedReply;

/* Equal CAN IRQ priorities give one RX producer; main is the consumer. */
static CAN_Packet requests[CAN_QUEUE_DEPTH], replies[CAN_QUEUE_DEPTH];
static volatile uint32_t requestHead, requestTail;
static volatile uint32_t replyHead, replyTail, eventHead, eventTail;
static uint32_t nextCache, lastHeartbeat, lastFast, lastPosition, lastRawPosition;
static CAN_Packet eventsQueue[CAN_QUEUE_DEPTH];
static uint8_t fastPending, positionPending, rawPositionPending;
static CAN_CachedReply cache[CAN_QUEUE_DEPTH];
static uint8_t heartbeatPending, faultPending, recoveryPrepared, sampleDivider;
static uint16_t lastReportedFault;
static uint8_t lastHealthyState, faultState, busOffOriginState;
static uint16_t canLegacyFaultDetail;
static volatile uint16_t canFaultBits;
static volatile uint8_t adcProgress;
static uint32_t recoveryTick;
static volatile uint32_t busOffTick;
static FDCAN_TxHeaderTypeDef txHeader;
/* Accepted in TIM3, but hardware reset is performed only from main CAN_Poll. */
static volatile uint8_t resetPhase;
static uint8_t resetRequest[8];
static uint32_t resetStarted, resetTxBuffer, resetAcknowledged;

static HAL_StatusTypeDef CAN_InitResult(HAL_StatusTypeDef result)
{
    if (result != HAL_OK) canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
    return result;
}

static uint16_t CAN_MotorFaultBits(void)
{
    switch (motor_fault_reason) {
    case MOTOR_FAULT_NONE: case MOTOR_FAULT_STOP: return 0U;
    case MOTOR_FAULT_ENCODER: return 1U << 6;
    case MOTOR_FAULT_ALIGN_TIMEOUT: case MOTOR_FAULT_ALIGN_UNSTABLE: return 1U << 9;
    case MOTOR_FAULT_ADC_RANGE: {
        uint16_t bits = 0U;
        if (motor_fault_snapshot.raw_a < CURRENT_ADC_MIN_CODE || motor_fault_snapshot.raw_a > CURRENT_ADC_MAX_CODE)
            bits |= 1U << 2;
        if (motor_fault_snapshot.raw_c < CURRENT_ADC_MIN_CODE || motor_fault_snapshot.raw_c > CURRENT_ADC_MAX_CODE)
            bits |= 1U << 3;
        return bits != 0U ? bits : (uint16_t)((1U << 2) | (1U << 3));
    }
    case MOTOR_FAULT_CALIBRATION: return 1U << 4;
    /* Legacy reason does not distinguish hard/soft overcurrent. */
    case MOTOR_FAULT_OVERCURRENT: return 1U << 0;
    case MOTOR_FAULT_DRIVER: return 1U << 5;
    case MOTOR_FAULT_NONFINITE: return 1U << 7;
    case MOTOR_FAULT_ADC_TIMEOUT: return 1U << 8;
    case MOTOR_FAULT_COMMAND_TIMEOUT: return 1U << 10;
    default: return 1U << 15; /* Bus-monitor/internal fault: detail stays in RAM. */
    }
}

static void CAN_ReadStatus(CAN_NodeStatus *status)
{
    memset(status, 0, sizeof(*status));
    status->commandAge = UINT32_MAX;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    status->faultBits = (uint16_t)(CAN_MotorFaultBits() | canFaultBits);
    if (motor_fault_reason != MOTOR_FAULT_NONE && motor_fault_reason != MOTOR_FAULT_STOP)
        status->state = CAN_STATE_FAULT;
    else if (canFaultBits != 0U) status->state = CAN_STATE_COMM_LOST;
    else if (motor_electrical_calibrated) status->state = CAN_STATE_READY;
    else if (motor_startup_cancelled) status->state = CAN_STATE_DISABLED;
    else if (alignment_in_progress) status->state = CAN_STATE_ALIGNING;
    else if (!motor_current.Calibrated) status->state = CAN_STATE_CALIBRATING;
    else status->state = CAN_STATE_DISABLED;
    status->mode = motor_control.mode;
    status->adcProgress = adcProgress;
    status->capabilities = 1U | ((uint32_t)APP_UART_ENABLE << 2) |
                           ((uint32_t)(motor_electrical_calibrated != 0U) << 5);
    status->busStatus = canDebug.tec | (canDebug.rec << 8) |
                        (canDebug.busOff << 16) | (canDebug.errorPassive << 17);
    __set_PRIMASK(primask);
    if (MotorSupervisor_Active()) MotorSupervisor_ReadStatus(status, HAL_GetTick());
    if (MotorSupervisor_Active()) status->capabilities |= CAN_CAP_SOFTWARE_RESET | CAN_CAP_ELECTRICAL_STORAGE;
    status->firmwareVersion = APP_FIRMWARE_VERSION;
    status->uid[0] = HAL_GetUIDw0();
    status->uid[1] = HAL_GetUIDw1();
    status->uid[2] = HAL_GetUIDw2();
}

static void CAN_Stop(void)
{
    if (MotorSupervisor_Active()) MotorSupervisor_StopISR(); else foc_disarm();
    canDebug.stopCount++;
}

static void CAN_BusOff(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (canDebug.recoveryPhase == 0U) {
        busOffOriginState = alignment_in_progress ? CAN_STATE_ALIGNING : lastHealthyState;
        if (MotorSupervisor_Active()) MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_BUS_OFF, 0U);
        else CAN_Stop();
        canFaultBits |= CAN_FAULT_BUS_OFF;
        busOffTick = HAL_GetTick();
        canDebug.busOffCount++;
        __DMB();
        canDebug.recoveryPhase = 1U;
    }
    __set_PRIMASK(primask);
}

static void CAN_Receive(uint32_t fifo, uint32_t events)
{
    uint32_t budget;
    if (!canDebug.active) return;
    if (events & (FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST))
        canDebug.rxLostCount++;
    for (budget = 0U; budget < 3U; budget++) {
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[64]; /* HAL copies the full DLC before we reject FD. */
        uint32_t i;
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, fifo) == 0U) break;
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, fifo, &header, data) != HAL_OK) {
            canDebug.rxReadFailedCount++;
            canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
            break;
        }
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.BitRateSwitch != FDCAN_BRS_OFF ||
            header.DataLength != FDCAN_DLC_BYTES_8) {
            canDebug.rxIgnoredCount++;
            continue;
        }
        canDebug.rxCount++;
        canDebug.lastRxId = header.Identifier;
        canDebug.lastRxDlc = header.DataLength;
        for (i = 0U; i < 8U; i++) canDebug.lastRxData[i] = data[i];
        if (header.Identifier == CAN_ID_DISARM) {
            uint8_t nonzero = 0U;
            for (i = 0U; i < 8U; i++) nonzero |= data[i];
            if (nonzero == 0U) CAN_Stop();
            else canDebug.rxIgnoredCount++;
        } else if (APP_CAN_NODE_ID != 0U && header.Identifier == CAN_ID_REQUEST + APP_CAN_NODE_ID) {
            /* Stop before queue checks: a full queue cannot delay shutdown. */
            if (CAN_ProtocolIsStop(data)) CAN_Stop();
            if ((uint32_t)(requestHead - requestTail) >= CAN_QUEUE_DEPTH) canDebug.requestQueueFullCount++;
            else {
                CAN_Packet *packet = &requests[requestHead % CAN_QUEUE_DEPTH];
                memcpy(packet->data, data, 8U);
                packet->epoch = MotorSupervisor_Epoch(); packet->receivedMs = HAL_GetTick();
                __DMB();
                requestHead++;
            }
        } else if (APP_CAN_NODE_ID && header.Identifier == CAN_ID_TARGET + APP_CAN_NODE_ID) {
            if (resetPhase || !MotorSupervisor_TargetISR(data, HAL_GetTick())) canDebug.rxIgnoredCount++;
        } else canDebug.rxIgnoredCount++;
    }
}

static void CAN_HandleRequest(const CAN_Packet *packet, const CAN_NodeStatus *status,
                               uint32_t now, uint8_t reply[8])
{
    const uint8_t *request = packet->data;
    uint32_t i;
    if (MotorCalibration_Busy() && !CAN_ProtocolIsStop(request) && request[0] != CAN_OP_GET_INFO &&
        request[0] != CAN_OP_GET_STATUS && request[0] != CAN_OP_GET_PARAM && request[0] != CAN_OP_SAVE_CONFIG) {
        memset(reply, 0, 8U); reply[0] = request[0]; reply[1] = request[1]; reply[2] = CAN_RESULT_BUSY; return;
    }
    if (resetPhase && !CAN_ProtocolIsStop(request) && request[0] != CAN_OP_GET_INFO &&
        request[0] != CAN_OP_GET_STATUS && request[0] != CAN_OP_GET_PARAM) {
        memset(reply, 0, 8U); reply[0] = request[0]; reply[1] = request[1];
        reply[2] = memcmp(request, resetRequest, 8U) == 0 ? CAN_RESULT_ACCEPTED : CAN_RESULT_BUSY;
        return;
    }
    if (MotorSupervisor_Active() && packet->epoch != MotorSupervisor_Epoch() && !CAN_ProtocolIsStop(request)) {
        memset(reply, 0, 8U); reply[0] = request[0]; reply[1] = request[1]; reply[2] = CAN_RESULT_BAD_SESSION; return;
    }
    if (!CAN_ProtocolIsStop(request)) {
        for (i = 0U; i < CAN_QUEUE_DEPTH; i++) {
            if (cache[i].valid && cache[i].epoch == packet->epoch && (uint32_t)(now - cache[i].created) < CAN_CACHE_TIME_MS &&
                cache[i].request[1] == request[1]) {
                if (memcmp(cache[i].request, request, 8U) == 0) memcpy(reply, cache[i].reply, 8U);
                else {
                    memset(reply, 0, 8U);
                    reply[0] = request[0]; reply[1] = request[1]; reply[2] = CAN_RESULT_CONFLICT;
                }
                return;
            }
        }
    }
    if (request[0] == CAN_OP_SOFTWARE_RESET && MotorSupervisor_Active()) {
        motor_health_t health;
        uint32_t key = MotorPort_Lock();
        MotorPort_ReadHealth(&health);
        memset(reply, 0, 8U); reply[0] = request[0]; reply[1] = request[1];
        if (CAN_ReadU16(request + 2) != CAN_RESET_CONFIRM) reply[2] = CAN_RESULT_BAD_VALUE;
        else if (!CAN_ReadU32(request + 4) || CAN_ReadU32(request + 4) != status->challenge)
            reply[2] = CAN_RESULT_BAD_SESSION;
        else if ((uint32_t)(now - packet->receivedMs) >= 100U) reply[2] = CAN_RESULT_STALE;
        else if (!MotorSupervisor_ResetReady() || health.outputEnabled || health.driverEnabled || alignment_in_progress ||
                 (status->state != CAN_STATE_DISABLED && status->state != CAN_STATE_READY &&
                  status->state != CAN_STATE_FAULT && status->state != CAN_STATE_COMM_LOST))
            reply[2] = CAN_RESULT_BAD_STATE;
        else {
            memcpy(resetRequest, request, 8U);
            resetStarted = now; resetTxBuffer = 0U;
            CAN_Stop(); /* Revoke the session and all queued target authority. */
            __DMB(); resetPhase = 1U;
            reply[2] = CAN_RESULT_ACCEPTED;
        }
        MotorPort_Unlock(key);
    } else if (request[0] == CAN_OP_GET_INFO || request[0] == CAN_OP_GET_STATUS || !MotorSupervisor_Active())
        CAN_ProtocolReply(request, status, reply);
    else if (request[0] == CAN_OP_CLEAR_FAULT &&
             (canDebug.busOff || canDebug.recoveryPhase)) {
        memset(reply, 0, 8U); reply[0] = request[0]; reply[1] = request[1]; reply[2] = CAN_RESULT_BAD_STATE;
    } else MotorSupervisor_Request(request, packet->epoch, packet->receivedMs, now, reply);
    if (!CAN_ProtocolIsStop(request)) {
        CAN_CachedReply *entry = &cache[nextCache++ % CAN_QUEUE_DEPTH];
        memcpy(entry->request, request, 8U);
        memcpy(entry->reply, reply, 8U);
        entry->created = now; entry->epoch = packet->epoch;
        entry->valid = 1U;
    }
}

/* HAL_Stop contains timeout polling. RM0440 bus-off recovery requests INIT=0
 * and observes completion, while cancelling every stale hardware TX request. */
static bool CAN_Recover(uint32_t now, const FDCAN_ProtocolStatusTypeDef *protocol)
{
    if (canDebug.recoveryPhase == 0U) return false;
    if (!recoveryPrepared) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        requestTail = requestHead;
        replyTail = replyHead; eventTail = eventHead;
        memset(cache, 0, sizeof(cache));
        __set_PRIMASK(primask);
        heartbeatPending = faultPending = fastPending = positionPending = rawPositionPending = 0U;
        recoveryPrepared = 1U;
        if (HAL_FDCAN_AbortTxRequest(&hfdcan1, CAN_TX_BUFFERS) != HAL_OK) {
            canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
            canDebug.recoveryPhase = 3U;
        }
    }
    if (canDebug.recoveryPhase == 3U) return true;
    if (canDebug.recoveryPhase == 1U && (uint32_t)(now - busOffTick) >= 100U) {
        CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
        canDebug.recoveryAttempts++;
        recoveryTick = now;
        canDebug.recoveryPhase = 2U;
        return true;
    }
    if (canDebug.recoveryPhase == 2U) {
        if (!protocol->BusOff && (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
            hfdcan1.Instance->TXBRP == 0U) {
            canDebug.recoveryPhase = 0U;
            recoveryPrepared = 0U;
            lastHeartbeat = now - CAN_HEARTBEAT_PERIOD_MS;
            faultPending = 1U;
            return false; /* Fault and startup inhibition remain latched. */
        }
        if ((uint32_t)(now - recoveryTick) >= 250U) {
            if (canDebug.recoveryAttempts >= 3U) canDebug.recoveryPhase = 3U;
            else { busOffTick = now; canDebug.recoveryPhase = 1U; }
        }
    }
    return true;
}

static void CAN_Send(const CAN_NodeStatus *status)
{
    uint8_t data[8];
    uint32_t kind;
    if (MotorCalibration_HoldsTx() || resetPhase >= 2U || canDebug.recoveryPhase != 0U || !APP_CAN_NODE_ID ||
        (!faultPending && replyHead == replyTail && eventHead == eventTail && !heartbeatPending && !fastPending && !positionPending && !rawPositionPending)) return;
    /* At most one periodic frame in hardware; reserve room for replies/faults. */
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) <=
        (uint32_t)((!faultPending && replyHead == replyTail && eventHead == eventTail) ? 2U : 0U)) {
        canDebug.txBusyCount++;
        return;
    }
    if (faultPending) {
        kind = 0U;
        txHeader.Identifier = CAN_ID_FAULT + APP_CAN_NODE_ID;
        data[0] = faultState; data[1] = status->mode;
        CAN_WriteU16(data + 2, status->faultBits);
        CAN_WriteU16(data + 4, 0U);
        CAN_WriteU16(data + 6, canLegacyFaultDetail);
        if (MotorSupervisor_Active()) {
            uint16_t seq, detail;
            MotorSupervisor_ReadFault(&data[0], &data[1], &seq, &detail);
            CAN_WriteU16(data + 4, seq); CAN_WriteU16(data + 6, detail);
        }
    } else if (eventHead != eventTail) {
        kind = 3U; txHeader.Identifier = CAN_ID_EVENT + APP_CAN_NODE_ID;
        __DMB(); memcpy(data, eventsQueue[eventTail % CAN_QUEUE_DEPTH].data, 8U);
    } else if (replyHead != replyTail) {
        kind = 1U;
        txHeader.Identifier = CAN_ID_REPLY + APP_CAN_NODE_ID;
        memcpy(data, replies[replyTail % CAN_QUEUE_DEPTH].data, 8U);
    } else if (heartbeatPending) {
        kind = 2U;
        txHeader.Identifier = CAN_ID_HEARTBEAT + APP_CAN_NODE_ID;
        CAN_ProtocolHeartbeat(status, data);
    }
    else {
        motor_snapshot_t sample;
        if (!MotorSupervisor_ReadSample(&sample) || !sample.valid ||
            (!sample.electricalValid && !(APP_MOTOR_ROLE == MOTOR_ROLE_JOINT && (positionPending || rawPositionPending))) ||
            (uint32_t)(HAL_GetTick() - sample.capturedMs) >= 2U) {
            fastPending = positionPending = rawPositionPending = 0U; return;
        }
        if (positionPending && APP_MOTOR_ROLE == MOTOR_ROLE_JOINT) {
                int32_t positionQ = 0;
                uint8_t coordinateValid = 0U, reason = 0U, dirty = 0U;
                uint16_t flags = 0U;
                kind = 6U; txHeader.Identifier = CAN_ID_JOINT_POSITION + APP_CAN_NODE_ID;
                (void)MotorSupervisor_ReadAxisStatus(&positionQ, &coordinateValid, &reason, &dirty);
                if (reason != MOTOR_AXIS_STATUS_NO_REFERENCE) flags |= 1U;
                if (coordinateValid) flags |= 1U << 1;
                if (reason == MOTOR_AXIS_STATUS_VALID) flags |= 1U << 2;
                if (dirty) flags |= 1U << 3;
                if (coordinateValid && MotorProfile_Selected.positionMarginQ != 0U &&
                    (int64_t)positionQ - MotorProfile_Selected.minimumQ <= MotorProfile_Selected.positionMarginQ)
                    flags |= 1U << 4;
                if (coordinateValid && MotorProfile_Selected.positionMarginQ != 0U &&
                    (int64_t)MotorProfile_Selected.maximumQ - positionQ <= MotorProfile_Selected.positionMarginQ)
                    flags |= 1U << 5;
                if (sample.appliedValid) flags |= 1U << 7;
                CAN_WriteU16(data, sample.appliedSeq);
                CAN_WriteU16(data + 2, flags);
                CAN_WriteU32(data + 4, coordinateValid ? (uint32_t)positionQ : 0U);
        } else if ((positionPending && APP_MOTOR_ROLE != MOTOR_ROLE_JOINT) || rawPositionPending) {
                kind = 5U; txHeader.Identifier = CAN_ID_POSITION + APP_CAN_NODE_ID;
                CAN_WriteU32(data, (uint32_t)sample.turns); CAN_WriteU16(data + 4, sample.rawAngle);
                CAN_WriteU16(data + 6, (uint16_t)sample.sequence);
        } else {
            if (!sample.electricalValid) { fastPending = 0U; return; }
            float speed = sample.velocity * 10.0f, iq = sample.iq * 1000.0f;
            uint8_t saturated = speed > 32767.0f || speed < -32768.0f || iq > 32767.0f || iq < -32768.0f;
            if (!isfinite(speed) || !isfinite(iq)) { fastPending = 0U; return; }
            speed = fmaxf(-32768.0f, fminf(speed, 32767.0f));
            iq = fmaxf(-32768.0f, fminf(iq, 32767.0f));
            kind = 4U; txHeader.Identifier = CAN_ID_FEEDBACK + APP_CAN_NODE_ID;
            data[0] = (uint8_t)sample.appliedSeq;
            data[1] = (status->state == CAN_STATE_ENABLED ? 1U : 0U) | (status->faultBits ? 2U : 0U) | 4U |
                (status->commandAge != UINT32_MAX && status->commandAge >= 10U ? 8U : 0U) |
                (sample.currentLimited ? 16U : 0U) | (sample.appliedValid && status->state == CAN_STATE_ENABLED ? 64U : 0U) |
                (saturated ? 128U : 0U);
            CAN_WriteU16(data + 2, sample.rawAngle);
            CAN_WriteU16(data + 4, (uint16_t)(int16_t)lroundf(speed)); CAN_WriteU16(data + 6, (uint16_t)(int16_t)lroundf(iq));
        }
    }
    canDebug.txAttemptCount++;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data) == HAL_OK) {
        canDebug.txQueuedCount++;
        if (resetPhase == 1U && kind == 1U && data[0] == CAN_OP_SOFTWARE_RESET &&
            data[1] == resetRequest[1] && data[2] == CAN_RESULT_ACCEPTED) {
            resetTxBuffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            resetPhase = 2U;
        }
        if (kind == 1U && data[0] == CAN_OP_SAVE_CONFIG && data[2] == CAN_RESULT_ACCEPTED)
            MotorCalibration_AcceptedTx(data[1], HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1));
        if (kind == 0U) faultPending = 0U;
        else if (kind == 1U) replyTail++;
        else if (kind == 2U) heartbeatPending = 0U;
        else if (kind == 3U) eventTail++;
        else if (kind == 4U) fastPending = 0U;
        else if (kind == 5U) {
            if (APP_MOTOR_ROLE == MOTOR_ROLE_JOINT) rawPositionPending = 0U;
            else positionPending = 0U;
        } else if (kind == 6U) positionPending = 0U;
    } else {
        canDebug.txFailedCount++;
        canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
    }
}
#endif

HAL_StatusTypeDef CAN_Init(void)
{
    canDebug = (CAN_DebugTypeDef){0};
#if APP_CAN_ENABLE
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef result;
    uint32_t i;
    canDebug.nodeId = APP_CAN_NODE_ID;
    if (hfdcan1.State != HAL_FDCAN_STATE_READY || hfdcan1.Init.StdFiltersNbr != 3U)
        return CAN_InitResult(HAL_ERROR);
    requestHead = requestTail = replyHead = replyTail = eventHead = eventTail = nextCache = 0U;
    resetPhase = 0U; resetTxBuffer = resetStarted = resetAcknowledged = 0U;
    fastPending = positionPending = rawPositionPending = 0U;
    lastFast = lastPosition = lastRawPosition = HAL_GetTick();
    heartbeatPending = faultPending = recoveryPrepared = sampleDivider = adcProgress = 0U;
    canFaultBits = lastReportedFault = 0U;
    lastHealthyState = faultState = busOffOriginState = CAN_STATE_BOOT;
    canLegacyFaultDetail = 0U;
    memset(cache, 0, sizeof(cache));
    canDebug.initStage = 1U;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID2 = 0x7FFU;
    for (i = 0U; i < 3U; i++) {
        filter.FilterIndex = i;
        filter.FilterID1 = i == 0U ? CAN_ID_DISARM :
                          (i == 1U ? CAN_ID_TARGET : CAN_ID_REQUEST) + APP_CAN_NODE_ID;
        filter.FilterConfig = i == 2U ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
        if (i != 0U && APP_CAN_NODE_ID == 0U) filter.FilterConfig = FDCAN_FILTER_DISABLE;
        result = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
        if (result != HAL_OK) return CAN_InitResult(result);
    }
    canDebug.initStage = 2U;
    result = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (result != HAL_OK) return CAN_InitResult(result);
    canDebug.initStage = 3U;
    result = HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
    if (result != HAL_OK) return CAN_InitResult(result);
    result = HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO1 | FDCAN_IT_GROUP_SMSG |
              FDCAN_IT_GROUP_BIT_LINE_ERROR | FDCAN_IT_GROUP_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE1);
    if (result != HAL_OK) return CAN_InitResult(result);
    canDebug.initStage = 4U;
    result = HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
          FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST |
          FDCAN_IT_TX_COMPLETE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
          CAN_TX_BUFFERS);
    if (result != HAL_OK) return CAN_InitResult(result);
    txHeader = (FDCAN_TxHeaderTypeDef){0};
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    canDebug.initStage = 5U;
    canDebug.active = 1U; /* Fully initialize before Start can trigger callbacks. */
    result = HAL_FDCAN_Start(&hfdcan1);
    if (result != HAL_OK) { canDebug.active = 0U; return CAN_InitResult(result); }
    lastHeartbeat = HAL_GetTick();
    canDebug.initStage = 6U;
#endif
    return HAL_OK;
}

void CAN_Poll(void)
{
#if APP_CAN_ENABLE
    CAN_NodeStatus status;
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef counters = {0};
    uint32_t now;
    if (!canDebug.active) return;
    now = HAL_GetTick();
    if (resetPhase) {
        if ((uint32_t)(now - resetStarted) >= 500U) {
            /* No confirmed reply means no reset. Keep the node stopped and
             * require a new challenge/request after failed transmission. */
            uint32_t key = MotorPort_Lock();
            if (resetTxBuffer) (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, resetTxBuffer);
            memset(cache, 0, sizeof(cache)); replyTail = replyHead;
            resetPhase = 0U;
            MotorPort_Unlock(key);
        } else if (resetPhase == 2U && resetTxBuffer &&
                   (hfdcan1.Instance->TXBTO & resetTxBuffer) != 0U &&
                   (hfdcan1.Instance->TXBRP & resetTxBuffer) == 0U) {
            resetAcknowledged = now; resetPhase = 3U;
        } else if (resetPhase == 3U && (uint32_t)(now - resetAcknowledged) >= 20U) {
            motor_health_t health;
            uint32_t key = MotorPort_Lock();
            MotorPort_ReadHealth(&health);
            if (!health.outputEnabled && !health.driverEnabled && !alignment_in_progress) {
                Motor_HardwareDisable();
                NVIC_SystemReset();
            }
            resetPhase = 0U; /* Host model can return; hardware reset cannot. */
            MotorPort_Unlock(key);
        }
    }
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) != HAL_OK ||
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &counters) != HAL_OK) {
        canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
        return;
    }
    canDebug.tec = counters.TxErrorCnt;
    canDebug.rec = counters.RxErrorCnt;
    canDebug.busOff = protocol.BusOff;
    canDebug.errorPassive = protocol.ErrorPassive;
    if (protocol.BusOff) CAN_BusOff();
    if (CAN_Recover(now, &protocol)) return;
    CAN_ReadStatus(&status);
    if (status.faultBits != lastReportedFault) {
        if (lastReportedFault == 0U && status.faultBits != 0U) {
            if (motor_fault_reason != MOTOR_FAULT_NONE && motor_fault_reason != MOTOR_FAULT_STOP) {
                faultState = motor_fault_snapshot.was_aligning ? CAN_STATE_ALIGNING : lastHealthyState;
                canLegacyFaultDetail = (uint16_t)motor_fault_reason;
            } else {
                faultState = busOffOriginState;
                canLegacyFaultDetail = 0U;
            }
        }
        lastReportedFault = status.faultBits;
        if (status.faultBits != 0U) faultPending = 1U;
    }
    if (status.faultBits == 0U) lastHealthyState = status.state;
    if (!MotorSupervisor_Active()) CAN_Tick1kHz();
    if ((uint32_t)(now - lastFast) >= CAN_FEEDBACK_PERIOD_MS) { lastFast = now; fastPending = 1U; }
    if ((uint32_t)(now - lastPosition) >= (APP_MOTOR_ROLE == MOTOR_ROLE_JOINT ? 50U : CAN_POSITION_PERIOD_MS)) {
        lastPosition = now; positionPending = 1U;
    }
    if (APP_MOTOR_ROLE == MOTOR_ROLE_JOINT &&
        (uint32_t)(now - lastRawPosition) >= CAN_POSITION_PERIOD_MS) {
        lastRawPosition = now; rawPositionPending = 1U;
    }
    if ((uint32_t)(now - lastHeartbeat) >= CAN_HEARTBEAT_PERIOD_MS) {
        lastHeartbeat = now;
        heartbeatPending = 1U;
    }
    CAN_Send(&status);
#endif
}

void CAN_Tick1kHz(void)
{
#if APP_CAN_ENABLE
    uint32_t now = HAL_GetTick(), budget;
    CAN_NodeStatus status;
    if (!canDebug.active || canDebug.recoveryPhase) return;
    for (budget = 0U; budget < 4U && requestTail != requestHead &&
         (uint32_t)(replyHead - replyTail) < CAN_QUEUE_DEPTH; budget++) {
        CAN_Packet packet;
        uint8_t reply[8];
        __DMB(); packet = requests[requestTail % CAN_QUEUE_DEPTH]; __DMB(); requestTail++;
        CAN_ReadStatus(&status);
        if (packet.data[0] == CAN_OP_CAL_KEEPALIVE) {
            MotorSupervisor_Request(packet.data, packet.epoch, packet.receivedMs, now, reply);
            continue; /* Lease stream: no reply and no management cache. */
        }
        CAN_HandleRequest(&packet, &status, now, replies[replyHead % CAN_QUEUE_DEPTH].data);
        __DMB(); replyHead++;
    }
    if ((uint32_t)(eventHead - eventTail) < CAN_QUEUE_DEPTH) {
        motor_event_t event;
        if (MotorCalibration_TakeEvent(&event) || MotorSupervisor_TakeEvent(&event)) {
            uint8_t *data = eventsQueue[eventHead % CAN_QUEUE_DEPTH].data;
            uint32_t i;
            data[0] = event.opcode; data[1] = event.requestId; data[2] = event.result; data[3] = event.detail;
            CAN_WriteU32(data + 4, event.value);
            for (i = 0U; i < CAN_QUEUE_DEPTH; i++) {
                if (cache[i].valid && cache[i].request[0] == event.opcode && cache[i].request[1] == event.requestId &&
                    cache[i].reply[2] == CAN_RESULT_ACCEPTED) {
                    memcpy(cache[i].reply, data, 8U);
                    if (event.opcode == CAN_OP_SAVE_CONFIG) cache[i].epoch = MotorSupervisor_Epoch();
                }
            }
            __DMB(); eventHead++;
        }
    }
#endif
}

void CAN_OnCurrentSample(void)
{
#if APP_CAN_ENABLE
    if (++sampleDivider >= 16U) { sampleDivider = 0U; adcProgress++; }
#endif
}

void CAN_StorageFlush(void)
{
#if APP_CAN_ENABLE
    uint32_t i, key = MotorPort_Lock();
    requestTail = requestHead; replyTail = replyHead;
    heartbeatPending = fastPending = positionPending = rawPositionPending = 0U;
    for (i = 0U; i < CAN_QUEUE_DEPTH; i++) {
        if (cache[i].request[0] != CAN_OP_SAVE_CONFIG) cache[i].valid = 0U;
    }
    if (canDebug.active && HAL_FDCAN_AbortTxRequest(&hfdcan1, CAN_TX_BUFFERS) != HAL_OK) {
        canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
        MotorSupervisor_FaultISR(MOTOR_FAULT_BIT_INTERNAL, 0xCBU);
    }
    MotorPort_Unlock(key);
#endif
}

#if APP_CAN_ENABLE
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t events)
{
    if (hfdcan == &hfdcan1) CAN_Receive(FDCAN_RX_FIFO0, events);
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t events)
{
    if (hfdcan == &hfdcan1) CAN_Receive(FDCAN_RX_FIFO1, events);
}
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t buffers)
{
    if (hfdcan == &hfdcan1 && canDebug.active) {
        if (buffers & FDCAN_TX_BUFFER0) canDebug.txCompleteCount++;
        if (buffers & FDCAN_TX_BUFFER1) canDebug.txCompleteCount++;
        if (buffers & FDCAN_TX_BUFFER2) canDebug.txCompleteCount++;
    }
}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t events)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    if (hfdcan != &hfdcan1 || !canDebug.active) return;
    if ((events & FDCAN_IT_BUS_OFF) && HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol) == HAL_OK &&
        protocol.BusOff) CAN_BusOff();
}
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1 && canDebug.active) canDebug.halError = HAL_FDCAN_GetError(hfdcan);
}
#endif
