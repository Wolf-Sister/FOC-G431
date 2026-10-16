#include "can_protocol.h"

uint16_t CAN_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

uint32_t CAN_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

int32_t CAN_ReadI32(const uint8_t *data)
{
    uint32_t raw = CAN_ReadU32(data);
    return raw <= INT32_MAX ? (int32_t)raw : -1 - (int32_t)(UINT32_MAX - raw);
}

void CAN_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void CAN_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

bool CAN_ProtocolIsStop(const uint8_t request[8])
{
    return (request[0] == CAN_OP_DISARM || request[0] == CAN_OP_HOME_ABORT) &&
           CAN_ReadU16(request + 2) == 0U && CAN_ReadU32(request + 4) == 0U;
}

void CAN_ProtocolReply(const uint8_t request[8], const CAN_NodeStatus *status,
                       uint8_t reply[8])
{
    uint16_t selector = CAN_ReadU16(request + 2);
    uint32_t value = 0U;
    uint8_t result = CAN_RESULT_OK;
    reply[0] = request[0];
    reply[1] = request[1];
    reply[3] = 0U;

    switch (request[0]) {
    case CAN_OP_GET_INFO:
        if (CAN_ReadU32(request + 4) != 0U) {
            result = CAN_RESULT_BAD_VALUE;
        } else {
            switch (selector) {
            case 0: value = CAN_PROTOCOL_VERSION; break;
            case 1: value = status->firmwareVersion; break;
            case 2: case 3: case 4: value = status->uid[selector - 2U]; break;
            case 5:
                value = status->challenge;
                if (!value) result = CAN_RESULT_BUSY;
                break;
            case 6: value = status->capabilities; break;
            case 7: value = status->configRevision; break;
            default: result = CAN_RESULT_BAD_INDEX; break;
            }
        }
        break;
    case CAN_OP_GET_STATUS:
        if (CAN_ReadU32(request + 4) != 0U) {
            result = CAN_RESULT_BAD_VALUE;
        } else {
            switch (selector) {
            case 0: value = status->state | ((uint32_t)status->mode << 8) |
                            ((uint32_t)status->role << 16) | ((uint32_t)status->source << 24); break;
            case 1: value = status->faultBits; break;
            case 2: value = status->acceptedSeq | ((uint32_t)status->appliedSeq << 16); break;
            case 3: value = status->commandAge; break;
            case 4: value = status->busStatus; break;
            case 5: value = 0U; break; /* HOME remains unavailable. */
            case 6: value = status->flags; break;
            default: result = CAN_RESULT_BAD_INDEX; break;
            }
        }
        break;
    case CAN_OP_DISARM: case CAN_OP_HOME_ABORT:
        if (!CAN_ProtocolIsStop(request)) result = CAN_RESULT_BAD_VALUE;
        break;
    default:
        result = CAN_RESULT_UNSUPPORTED;
        break;
    }
    reply[2] = result;
    CAN_WriteU32(reply + 4, value);
}

void CAN_ProtocolHeartbeat(const CAN_NodeStatus *status, uint8_t data[8])
{
    data[0] = status->state;
    data[1] = status->mode;
    CAN_WriteU16(data + 2, status->faultBits);
    CAN_WriteU16(data + 4, status->appliedSeq);
    data[6] = status->adcProgress;
    data[7] = status->commandAge > 255U ? 255U : (uint8_t)status->commandAge;
}
