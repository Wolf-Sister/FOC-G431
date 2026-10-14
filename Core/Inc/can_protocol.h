#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_PROTOCOL_VERSION 0x00010000U
#define CAN_ID_DISARM        0x001U
#define CAN_ID_FAULT         0x080U
#define CAN_ID_TARGET        0x120U
#define CAN_ID_REQUEST       0x300U
#define CAN_ID_REPLY         0x380U
#define CAN_ID_HEARTBEAT     0x700U
#define CAN_OP_GET_INFO      0x01U
#define CAN_OP_GET_STATUS    0x02U
#define CAN_OP_DISARM        0x12U
#define CAN_OP_HOME_ABORT    0x1AU
#define CAN_RESULT_OK          0U
#define CAN_RESULT_BAD_VALUE   3U
#define CAN_RESULT_BAD_INDEX   4U
#define CAN_RESULT_CONFLICT    9U
#define CAN_RESULT_UNSUPPORTED 11U

typedef enum {
    CAN_STATE_BOOT = 0,
    CAN_STATE_DISABLED = 1,
    CAN_STATE_CALIBRATING = 2,
    CAN_STATE_ALIGNING = 3,
    CAN_STATE_READY = 4,
    CAN_STATE_FAULT = 7,
    CAN_STATE_COMM_LOST = 8
} CAN_NodeState;

typedef struct {
    uint8_t state, mode, adcProgress;
    uint16_t faultBits;
    uint32_t firmwareVersion, uid[3], capabilities, busStatus;
} CAN_NodeStatus;

uint16_t CAN_ReadU16(const uint8_t *data);
uint32_t CAN_ReadU32(const uint8_t *data);
void CAN_WriteU16(uint8_t *data, uint16_t value);
void CAN_WriteU32(uint8_t *data, uint32_t value);
bool CAN_ProtocolIsStop(const uint8_t request[8]);
void CAN_ProtocolReply(const uint8_t request[8], const CAN_NodeStatus *status,
                       uint8_t reply[8]);
void CAN_ProtocolHeartbeat(const CAN_NodeStatus *status, uint8_t data[8]);

#endif
