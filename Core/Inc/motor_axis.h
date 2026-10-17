#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_AXIS_ENCODER_COUNTS 16384U
#define MOTOR_AXIS_TURN_Q 62832LL /* 2*pi in units of 0.0001 rad. */

enum {
    MOTOR_AXIS_STATUS_VALID = 0U,
    MOTOR_AXIS_STATUS_NO_REFERENCE = 1U,
    MOTOR_AXIS_STATUS_LIMITS_INVALID = 2U,
    MOTOR_AXIS_STATUS_STALE = 3U,
    MOTOR_AXIS_STATUS_OUTSIDE_RANGE = 4U,
    MOTOR_AXIS_STATUS_AMBIGUOUS = 5U
};

typedef struct {
    int32_t direction;       /* +1 or -1: raw encoder direction to joint direction. */
    uint16_t referenceRaw;   /* AS5047P single-turn count, 0..16383. */
    int32_t referenceQ;       /* Joint angle at referenceRaw, units of 0.0001 rad. */
    int32_t minimumQ;
    int32_t maximumQ;
    uint8_t referencePresent;
} motor_axis_config_t;

typedef enum {
    MOTOR_AXIS_OK = 0,
    MOTOR_AXIS_INVALID_CONFIG,
    MOTOR_AXIS_INVALID_RAW,
    MOTOR_AXIS_OUTSIDE_RANGE,
    MOTOR_AXIS_AMBIGUOUS
} motor_axis_result_t;

typedef struct {
    uint32_t startedMs, lastSampleMs, lastSequence;
    int32_t originRaw, minDelta, maxDelta;
    int64_t sumDelta;
    uint16_t sampleCount;
    uint8_t active;
} motor_axis_zero_window_t;

bool MotorAxis_ConfigValid(const motor_axis_config_t *config);
void MotorAxis_ZeroWindowReset(motor_axis_zero_window_t *window);
void MotorAxis_ZeroWindowAdd(motor_axis_zero_window_t *window,
                             uint32_t nowMs,
                             uint32_t encoderSequence,
                             uint16_t raw,
                             bool sampleValid);
bool MotorAxis_ZeroWindowRead(const motor_axis_zero_window_t *window,
                              uint32_t nowMs,
                              uint16_t *meanRaw);
motor_axis_result_t MotorAxis_MapRaw(const motor_axis_config_t *config,
                                     uint16_t raw,
                                     int32_t *jointQ,
                                     int8_t *branch); /* branch is k in the coordinate equation. */
int64_t MotorAxis_QToCounts(int32_t jointQ);
bool MotorAxis_JointSpeedToRaw(int32_t direction, float jointSpeed, float *rawSpeed);

#endif
