#ifndef MOTOR_MOTION_H
#define MOTOR_MOTION_H
#include <stdint.h>
#include <stdbool.h>
#define MOTOR_MOTION_LOOP_DT_S 0.002f /* 500 Hz outer-loop interval */
typedef struct {
    uint32_t speedLimit, positionSpeed, acceleration; /* 0.001 SI units */
    int32_t positionMin, positionMax; /* 0.0001 rad, current-boot raw axis */
} motor_motion_config_t;
typedef struct {
    motor_motion_config_t config;
    int64_t targetCounts;
    float targetRemainder, speed;
    int32_t target;
    uint8_t mode, failure;
} motor_motion_t;
enum {
    MOTION_FAILURE_NONE = 0, MOTION_FAILURE_OVERSPEED = 1,
    MOTION_FAILURE_POSITION = 2, MOTION_FAILURE_CONFIG = 3,
    MOTION_FAILURE_NONFINITE = 4, MOTION_FAILURE_ENCODER = 5
};
typedef struct {
    uint32_t tick, cycle;
    int64_t counts;
    int32_t target;
    float rawSpeed, filteredSpeed, speedReference;
    float integral, iqReference, iqMeasured;
    uint8_t mode, detail, valid;
} motor_motion_fault_snapshot_t;
int64_t MotorMotion_Counts(int32_t turns, uint16_t raw);
bool MotorMotion_ConfigValid(const motor_motion_config_t *c, uint8_t mode);
bool MotorMotion_TargetValid(const motor_motion_config_t *c, uint8_t mode, int32_t value, uint32_t iqLimit);
bool MotorMotion_InRange(const motor_motion_config_t *c, int64_t counts);
void MotorMotion_SetTarget(motor_motion_t *s, int32_t target);
float MotorMotion_Error(int64_t counts, int32_t position);
/* One MOTOR_MOTION_LOOP_DT_S step. False on invalid input, overspeed or position excursion. */
bool MotorMotion_Step(motor_motion_t *s, int64_t counts, float velocity, float kp, float *speed);
#endif
