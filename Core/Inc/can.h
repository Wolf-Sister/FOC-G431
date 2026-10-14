#ifndef CAN_H
#define CAN_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_TEST_TX_ID        0x124U
#define CAN_TEST_TX_PERIOD_MS 2000U

typedef struct {
    uint32_t initStage; /* 0: peripheral check, 1: filter, 2: global filter,
                        * 3: start, 4: running. */
    uint32_t active;
    uint32_t txAttemptCount;
    uint32_t txQueuedCount; /* HAL accepted the request; this is not an ACK count. */
    uint32_t txFailedCount;
    uint32_t txBusyCount;
    uint32_t rxCount;
    uint32_t rxReadFailedCount;
    uint32_t rxIgnoredCount;
    uint32_t halError;
    uint32_t lastRxId;
    uint32_t lastRxDlc;
    uint8_t lastRxData[8];
} CAN_DebugTypeDef;

extern volatile CAN_DebugTypeDef canDebug;

/* Call once after MX_FDCAN1_Init(). A failed initialization leaves polling idle. */
HAL_StatusTypeDef CAN_Init(void);
/* Main-loop only: submit at most one TX and read at most one RX per call. */
void CAN_Poll(void);

#ifdef __cplusplus
}
#endif

#endif
