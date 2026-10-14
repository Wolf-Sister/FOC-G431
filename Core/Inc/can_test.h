#ifndef CAN_TEST_H
#define CAN_TEST_H

#include "main.h"
#include <stdint.h>

/* Fixed test frame from the user's adapter settings; independent of node ID. */
#define CAN_TEST_HEARTBEAT_ID 0x001U
#define CAN_TEST_PERIOD_MS 1000U

/* Main-loop services only. No receive or motor-command handling. */
void CAN_Test_Init(void);
void CAN_Test_Poll(void);

typedef struct {
    uint32_t submitted; /* Accepted by TX FIFO, not proof of a bus ACK. */
    uint32_t completed;
    uint32_t dropped;
    uint32_t errors;
    uint32_t bus_off_events;
    uint32_t recovery_requests;
    uint8_t tx_error_count, rx_error_count;
    uint8_t last_error, bus_off, active;
} can_test_stats_t;

/* Written only by main; inspect through debugger. */
extern volatile can_test_stats_t can_test_stats;

#endif
