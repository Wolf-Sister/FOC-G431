#ifndef CAN_H
#define CAN_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_HEARTBEAT_PERIOD_MS 50U
#define CAN_FEEDBACK_PERIOD_MS 20U
#define CAN_POSITION_PERIOD_MS 200U

typedef struct {
    uint32_t initStage; /* 0: check, 1: filters, 2: global, 3: lines,
                        * 4: notification, 5: start, 6: running. */
    uint32_t active;
    uint32_t nodeId;
    uint32_t txAttemptCount;
    uint32_t txQueuedCount; /* HAL accepted the request; this is not an ACK count. */
    uint32_t txFailedCount;
    uint32_t txBusyCount;
    uint32_t rxCount;
    uint32_t rxReadFailedCount;
    uint32_t rxIgnoredCount;
    uint32_t rxLostCount;
    uint32_t requestQueueFullCount;
    uint32_t txCompleteCount;
    uint32_t stopCount;
    uint32_t busOffCount;
    uint32_t recoveryAttempts;
    uint32_t recoveryPhase; /* 0: normal, 1: cooldown, 2: wait bus, 3: exhausted. */
    uint32_t tec, rec, busOff, errorPassive;
    uint32_t halError;
    uint32_t lastRxId;
    uint32_t lastRxDlc;
    uint8_t lastRxData[8];
} CAN_DebugTypeDef;

extern volatile CAN_DebugTypeDef canDebug;

/* Call once after MX_FDCAN1_Init(). A failed initialization leaves polling idle. */
HAL_StatusTypeDef CAN_Init(void);
/* Main-loop only: bounded management/TX/recovery. RX uses HAL callbacks. */
void CAN_Poll(void);
/* TIM3 supervisor context; consumes at most four management requests. */
void CAN_Tick1kHz(void);
/* ADC callback only; counts actual samples, not timer or main-loop activity. */
void CAN_OnCurrentSample(void);

#ifdef __cplusplus
}
#endif

#endif
