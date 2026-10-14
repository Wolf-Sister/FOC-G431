#include "can.h"
#include "fdcan.h"

volatile CAN_DebugTypeDef canDebug;

#if APP_CAN_ENABLE
static const uint8_t canTestData[8] =
    {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
static FDCAN_TxHeaderTypeDef canTxHeader;
static uint32_t lastCanTxTick;

static HAL_StatusTypeDef CAN_InitResult(HAL_StatusTypeDef result)
{
    if (result != HAL_OK) {
        canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
    }
    return result;
}
#endif

HAL_StatusTypeDef CAN_Init(void)
{
    canDebug = (CAN_DebugTypeDef){0};
#if APP_CAN_ENABLE
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef result;

    if (hfdcan1.State != HAL_FDCAN_STATE_READY || hfdcan1.Init.StdFiltersNbr < 1U) {
        return CAN_InitResult(HAL_ERROR);
    }

    canDebug.initStage = 1U;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    /* A zero mask accepts every standard ID into FIFO0. */
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    result = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    if (result != HAL_OK) return CAN_InitResult(result);

    canDebug.initStage = 2U;
    result = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (result != HAL_OK) return CAN_InitResult(result);

    canTxHeader = (FDCAN_TxHeaderTypeDef){0};
    canTxHeader.Identifier = CAN_TEST_TX_ID;
    canTxHeader.IdType = FDCAN_STANDARD_ID;
    canTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    canTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    canTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    canTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    canTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    canTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    canDebug.initStage = 3U;
    result = HAL_FDCAN_Start(&hfdcan1);
    if (result != HAL_OK) return CAN_InitResult(result);

    lastCanTxTick = HAL_GetTick();
    canDebug.initStage = 4U;
    canDebug.active = 1U;
#endif
    return HAL_OK;
}

void CAN_Poll(void)
{
#if APP_CAN_ENABLE
    uint32_t now;
    if (canDebug.active == 0U) return;

    now = HAL_GetTick();
    if ((uint32_t)(now - lastCanTxTick) >= CAN_TEST_TX_PERIOD_MS) {
        /* Schedule from now so a delayed main loop never bursts to catch up. */
        lastCanTxTick = now;
        canDebug.txAttemptCount++;
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U) {
            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &canTxHeader, canTestData) == HAL_OK) {
                canDebug.txQueuedCount++;
            } else {
                canDebug.txFailedCount++;
                canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
            }
        } else {
            canDebug.txBusyCount++;
        }
    }

    /* Read one message per call to bound work under heavy bus traffic. */
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef header = {0};
        /* G4 HAL copies the DLC's full length, up to 64 bytes for an FD frame. */
        uint8_t data[64];
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            canDebug.rxReadFailedCount++;
            canDebug.halError = HAL_FDCAN_GetError(&hfdcan1);
        } else if (header.IdType != FDCAN_STANDARD_ID || header.Identifier > 0x7FFU ||
                   header.RxFrameType != FDCAN_DATA_FRAME || header.FDFormat != FDCAN_CLASSIC_CAN ||
                   header.DataLength > FDCAN_DLC_BYTES_8) {
            canDebug.rxIgnoredCount++;
        } else {
            uint32_t i;
            canDebug.lastRxId = header.Identifier;
            canDebug.lastRxDlc = header.DataLength;
            /* For Classic CAN the G4 HAL's unshifted DLC equals the byte count. */
            for (i = 0U; i < sizeof(canDebug.lastRxData); i++) {
                canDebug.lastRxData[i] = (i < header.DataLength) ? data[i] : 0U;
            }
            canDebug.rxCount++;
        }
    }
#endif
}
