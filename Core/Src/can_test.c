#include "can_test.h"
#include "fdcan.h"
#include "foc.h"
#include <stdio.h>

volatile can_test_stats_t can_test_stats;

#if APP_CAN_ENABLE
#define CAN_TEST_SERVICE_PERIOD_MS 50U
static uint32_t last_heartbeat_ms, last_service_ms, recovery_ms;
static uint8_t recovery_attempts, recovering;
static uint32_t pending_buffer;
#if APP_UART_ENABLE
static uint32_t last_diagnostic_ms;

static void CAN_Test_Report(uint32_t now)
{
    char log[224];
    uint32_t init = 0U, pending = 0U;
    int length;
    if ((uint32_t)(now - last_diagnostic_ms) < 1000U) return;
    last_diagnostic_ms = now;
    if (hfdcan1.Instance != NULL) {
        init = (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) ? 1U : 0U;
        if (pending_buffer)
            pending = HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, pending_buffer);
    }
    length = snprintf(log, sizeof(log),
        "[CAN] tx=%lu ok=%lu drop=%lu err=%lu TEC=%u REC=%u LEC=%u BO=%u active=%u pend=%lu init=%lu hal=%lX\r\n",
        (unsigned long)can_test_stats.submitted, (unsigned long)can_test_stats.completed,
        (unsigned long)can_test_stats.dropped, (unsigned long)can_test_stats.errors,
        (unsigned)can_test_stats.tx_error_count, (unsigned)can_test_stats.rx_error_count,
        (unsigned)can_test_stats.last_error, (unsigned)can_test_stats.bus_off,
        (unsigned)can_test_stats.active, (unsigned long)pending, (unsigned long)init,
        (unsigned long)hfdcan1.ErrorCode);
    if (length > 0 && (size_t)length < sizeof(log)) UART2_SendString(log);
    if (hfdcan1.Instance != NULL) {
        uint32_t pins = GPIOA->IDR;
        /* Read GPIO and TX registers only; another PSR read would clear LEC. */
        length = snprintf(log, sizeof(log),
            "[CAN-HW] TXD=%u RXD=%u MODE=%u/%u AF=%u/%u CCCR=%lX TXFQS=%lX TXBRP=%lX\r\n",
            (unsigned)((pins >> 12U) & 1U), (unsigned)((pins >> 11U) & 1U),
            (unsigned)((GPIOA->MODER >> 24U) & 3U), (unsigned)((GPIOA->MODER >> 22U) & 3U),
            (unsigned)((GPIOA->AFR[1] >> 16U) & 15U), (unsigned)((GPIOA->AFR[1] >> 12U) & 15U),
            (unsigned long)hfdcan1.Instance->CCCR,
            (unsigned long)hfdcan1.Instance->TXFQS,
            (unsigned long)hfdcan1.Instance->TXBRP);
        if (length > 0 && (size_t)length < sizeof(log)) UART2_SendString(log);
    }
}
#endif

void CAN_Test_Init(void)
{
    can_test_stats = (can_test_stats_t){0};
    pending_buffer = 0U;
    recovery_attempts = recovering = 0U;
    last_heartbeat_ms = HAL_GetTick();
    last_service_ms = last_heartbeat_ms;
#if APP_UART_ENABLE
    last_diagnostic_ms = last_heartbeat_ms;
#endif
    if (hfdcan1.State != HAL_FDCAN_STATE_READY ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        can_test_stats.errors++;
        UART2_SendString("[CAN] ERROR: heartbeat service not started\r\n");
        return;
    }
    can_test_stats.active = 1U;
    UART2_SendString("[CAN] OK: Classic 1M, test 1000ms; ID=0x001 data=11 22 33 44 55 66 77 88\r\n");
}

void CAN_Test_Poll(void)
{
    uint32_t now;
    FDCAN_ProtocolStatusTypeDef status;
    FDCAN_ErrorCountersTypeDef counters;
    FDCAN_TxHeaderTypeDef header = {0};
    const uint8_t data[8] = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0x66U, 0x77U, 0x88U};
    uint8_t send_due;

    now = HAL_GetTick();
#if APP_UART_ENABLE
    /* Keep reporting even when recovery or service startup has failed. */
    CAN_Test_Report(now);
#endif
    if (!can_test_stats.active) return;
    send_due = ((uint32_t)(now - last_heartbeat_ms) >= CAN_TEST_PERIOD_MS);
    if (!send_due && (uint32_t)(now - last_service_ms) < CAN_TEST_SERVICE_PERIOD_MS) return;
    last_service_ms = now;
    /* Keep error handling responsive; never catch up with a burst of old frames. */
    if (send_due) last_heartbeat_ms = now;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK ||
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &counters) != HAL_OK) {
        can_test_stats.errors++;
        return;
    }
    /* Reading PSR resets LEC to 7 (no change). Preserve the last observation. */
    if (status.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
        can_test_stats.last_error = (uint8_t)status.LastErrorCode;
    can_test_stats.tx_error_count = (uint8_t)counters.TxErrorCnt;
    can_test_stats.rx_error_count = (uint8_t)counters.RxErrorCnt;
    can_test_stats.bus_off = (uint8_t)status.BusOff;
    if (status.BusOff) {
        if (!recovering) {
            recovering = 1U;
            recovery_attempts = 0U;
            recovery_ms = now;
            can_test_stats.bus_off_events++;
            UART2_SendString("[CAN] bus-off: heartbeat paused\r\n");
        }
        if ((uint32_t)(now - recovery_ms) >= 100U) {
            if (recovery_attempts >= 3U) {
                can_test_stats.active = 0U;
                can_test_stats.errors++;
                UART2_SendString("[CAN] recovery exhausted; reset CAN test\r\n");
                return;
            }
            if (HAL_FDCAN_AbortTxRequest(&hfdcan1,
                FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
                can_test_stats.errors++;
            pending_buffer = 0U;
            /* Bus-off sets INIT in hardware while HAL remains BUSY. Clear it
             * to request the controller's recessive-bit recovery; never wait. */
            CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
            recovery_ms = now;
            recovery_attempts++;
            can_test_stats.recovery_requests++;
        }
        return;
    }
    if (recovering) {
        recovering = 0U;
        UART2_SendString("[CAN] bus recovered\r\n");
    }
    if (pending_buffer) {
        if (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, pending_buffer)) {
            if (HAL_FDCAN_AbortTxRequest(&hfdcan1, pending_buffer) != HAL_OK)
                can_test_stats.errors++;
            return; /* Cancellation is asynchronous; do not fill more slots. */
        }
        if (hfdcan1.Instance->TXBTO & pending_buffer) can_test_stats.completed++;
        else can_test_stats.dropped++; /* One-shot attempt without successful ACK. */
        pending_buffer = 0U;
    }
    if (!send_due) return;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        can_test_stats.dropped++;
        return;
    }
    header.Identifier = CAN_TEST_HEARTBEAT_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK) {
        pending_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
        can_test_stats.submitted++;
    } else {
        can_test_stats.errors++;
    }
}
#else
void CAN_Test_Init(void) {}
void CAN_Test_Poll(void) {}
#endif
