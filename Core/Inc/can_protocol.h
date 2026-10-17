#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_PROTOCOL_VERSION 0x00010100U
#define CAN_ID_DISARM        0x001U
#define CAN_ID_FAULT         0x080U
#define CAN_ID_TARGET        0x120U
#define CAN_ID_FEEDBACK      0x180U
#define CAN_ID_REQUEST       0x300U
#define CAN_ID_REPLY         0x380U
#define CAN_ID_EVENT         0x400U
#define CAN_ID_POSITION      0x500U
#define CAN_ID_JOINT_POSITION 0x200U
#define CAN_ID_HEARTBEAT     0x700U
#define CAN_OP_GET_INFO      0x01U
#define CAN_OP_GET_STATUS    0x02U
#define CAN_OP_OPEN_SESSION  0x03U
#define CAN_OP_SET_MODE      0x04U
#define CAN_OP_SET_SOURCE    0x05U
#define CAN_OP_CALIBRATE     0x10U
#define CAN_OP_ARM           0x11U
#define CAN_OP_DISARM        0x12U
#define CAN_OP_GET_PARAM     0x13U
#define CAN_OP_SET_PARAM     0x14U
#define CAN_OP_SET_IQ_LIMIT  0x15U
#define CAN_OP_SET_MECHANICAL_ZERO 0x16U
#define CAN_OP_SAVE_CONFIG   0x17U
#define CAN_OP_CAL_KEEPALIVE 0x1BU
#define CAN_OP_CLEAR_FAULT   0x1CU
#define CAN_OP_SOFTWARE_RESET 0x1DU
#define CAN_RESET_CONFIRM    0xA55AU
#define CAN_CAP_SOFTWARE_RESET (1UL << 8)
#define CAN_CAP_ELECTRICAL_STORAGE (1UL << 9)
#define CAN_CAL_SAVE_CONFIRM 0xECA1U
#define CAN_CAL_INVALIDATE_CONFIRM 0xECA0U
#define CAN_OP_HOME_ABORT    0x1AU
#define CAN_RESULT_OK          0U
#define CAN_RESULT_ACCEPTED    1U
#define CAN_RESULT_BAD_STATE   2U
#define CAN_RESULT_BAD_VALUE   3U
#define CAN_RESULT_BAD_INDEX   4U
#define CAN_RESULT_NOT_CONFIGURED 5U
#define CAN_RESULT_FAULT_LATCHED 6U
#define CAN_RESULT_BAD_SESSION 7U
#define CAN_RESULT_BUSY        8U
#define CAN_RESULT_CONFLICT    9U
#define CAN_RESULT_STALE       10U
#define CAN_RESULT_UNSUPPORTED 11U
#define CAN_RESULT_INTERNAL    12U

typedef enum {
    CAN_STATE_BOOT = 0,
    CAN_STATE_DISABLED = 1,
    CAN_STATE_CALIBRATING = 2,
    CAN_STATE_ALIGNING = 3,
    CAN_STATE_READY = 4,
    CAN_STATE_ENABLED = 5,
    CAN_STATE_FAULT = 7,
    CAN_STATE_COMM_LOST = 8
} CAN_NodeState;

typedef struct {
    uint8_t state, mode, adcProgress, role, source, flags;
    uint16_t faultBits, acceptedSeq, appliedSeq;
    uint32_t commandAge, challenge, configRevision;
    uint32_t firmwareVersion, uid[3], capabilities, busStatus;
} CAN_NodeStatus;

uint16_t CAN_ReadU16(const uint8_t *data);
uint32_t CAN_ReadU32(const uint8_t *data);
int32_t CAN_ReadI32(const uint8_t *data);
void CAN_WriteU16(uint8_t *data, uint16_t value);
void CAN_WriteU32(uint8_t *data, uint32_t value);
bool CAN_ProtocolIsStop(const uint8_t request[8]);
void CAN_ProtocolReply(const uint8_t request[8], const CAN_NodeStatus *status,
                       uint8_t reply[8]);
void CAN_ProtocolHeartbeat(const CAN_NodeStatus *status, uint8_t data[8]);

#endif
