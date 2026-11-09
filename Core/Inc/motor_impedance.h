#ifndef MOTOR_IMPEDANCE_H
#define MOTOR_IMPEDANCE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_IMPEDANCE_KP_DEFAULT_MILLI_A_PER_RAD 0U
#define MOTOR_IMPEDANCE_KD_DEFAULT_MILLI_A_S_PER_RAD 0U
#define MOTOR_IMPEDANCE_KP_MAX_MILLI_A_PER_RAD 10000U
#define MOTOR_IMPEDANCE_KD_MAX_MILLI_A_S_PER_RAD 2000U
#define MOTOR_IMPEDANCE_LOOP_DT_S 0.001f
#define MOTOR_IMPEDANCE_SPEED_LPF_ALPHA 0.4665119f

#define MOTOR_IMPEDANCE_FLAG_VALID       (1U << 0)
#define MOTOR_IMPEDANCE_FLAG_CLAMPED     (1U << 1)
#define MOTOR_IMPEDANCE_FLAG_DIAG_OVERFLOW (1U << 2)

typedef struct {
    uint32_t kpMilliAPerRad;       /* 0.001 A/rad */
    uint32_t kdMilliASPerRad;      /* 0.001 A*s/rad */
    uint16_t transitionMs;        /* Paired linear gain transition, 0..1000 ms. */
    int32_t feedforwardIqMa;      /* Electrical Iq mA, added AFTER direction mapping. */
    uint16_t feedforwardTransitionMs;
} motor_impedance_config_t;

typedef struct {
    int64_t targetCounts;
    float targetRemainder;
    float velocityFiltered;
    uint16_t targetSequence;
    uint8_t targetValid;
    uint8_t filterInitialized;
    uint8_t gainsInitialized;
    uint16_t gainStepsLeft, requestedTransitionMs;
    uint32_t requestedKp, requestedKd;
    float kpApplied, kdApplied, kpStep, kdStep;
    uint8_t feedforwardInitialized;
    uint16_t feedforwardStepsLeft, requestedFeedforwardMs;
    int32_t requestedFeedforwardMa;
    float feedforwardAppliedMa, feedforwardStep;
} motor_impedance_state_t;

typedef struct {
    float positionErrorRad;
    float filteredVelocityRadS;
    float iqCommandA;
    int32_t positionErrorQ;
    int32_t filteredVelocityMilliRadS;
    int32_t pCurrentMa;
    int32_t dCurrentMa;
    int32_t rawAxisCurrentMa;
    int32_t commandIqMa;
    int32_t feedforwardIqMa, rawElectricalMa;
    uint16_t targetSequence;
    uint8_t flags;
    uint8_t failure;
} motor_impedance_output_t;

typedef enum {
    MOTOR_IMPEDANCE_OK = 0,
    MOTOR_IMPEDANCE_BAD_CONFIG = 1,
    MOTOR_IMPEDANCE_BAD_TARGET = 2,
    MOTOR_IMPEDANCE_BAD_INPUT = 3,
    MOTOR_IMPEDANCE_BAD_DIRECTION = 4,
    MOTOR_IMPEDANCE_BAD_LIMIT = 5
} motor_impedance_result_t;

bool MotorImpedance_ConfigValid(const motor_impedance_config_t *config);
bool MotorImpedance_ConfigActive(const motor_impedance_config_t *config);
void MotorImpedance_Clear(motor_impedance_state_t *state);
bool MotorImpedance_Reset(motor_impedance_state_t *state, float measuredVelocityRadS);
bool MotorImpedance_SetTarget(motor_impedance_state_t *state, int32_t targetQ, uint16_t sequence);
motor_impedance_result_t MotorImpedance_Step(
    motor_impedance_state_t *state, const motor_impedance_config_t *config,
    int64_t measuredCounts, float measuredVelocityRadS, int8_t currentDirection,
    uint32_t currentLimitMa, motor_impedance_output_t *output);

#endif
