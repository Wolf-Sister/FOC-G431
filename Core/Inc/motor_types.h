#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H
#include <stdint.h>
#include <stdbool.h>
#include "motor_motion.h"

typedef struct {
    uint32_t sequence, capturedMs, encoderSequence;
    int32_t turns;
    uint16_t rawAngle, appliedSeq, rawA, rawC;
    float velocity, iq, id, maxPhaseCurrent;
    uint8_t valid, electricalValid, outputEnabled, appliedValid, currentLimited;
} motor_snapshot_t;

typedef struct {
    uint8_t currentCalibrated, electricalCalibrated, encoderValid;
    uint8_t busValid, driverFault, driverEnabled, outputEnabled;
    float busVoltage;
} motor_health_t;

typedef struct {
    uint8_t opcode, requestId, result, detail;
    uint32_t value;
} motor_event_t;

typedef enum { MOTOR_CMD_OFF, MOTOR_CMD_BEGIN_DRIVER, MOTOR_CMD_ALIGN,
               MOTOR_CMD_ARM, MOTOR_CMD_TARGET } motor_command_kind_t;

typedef struct {
    uint32_t version, epoch, receivedMs;
    uint16_t seq;
    uint8_t kind;
    int32_t iqMa; /* Legacy field name: mode 0=mA, 1=mrad/s, 2=0.0001 rad. */
    uint32_t currentLimitMa;
    float alignmentVoltage;
    uint8_t mode;
    motor_motion_config_t motion;
} motor_command_t;
#endif
