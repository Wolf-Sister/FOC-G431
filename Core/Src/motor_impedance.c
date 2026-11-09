#include "motor_impedance.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#define MOTOR_IMPEDANCE_RAD_PER_COUNT (6.283185307179586476925286766559 / 16384.0)

bool MotorImpedance_ConfigValid(const motor_impedance_config_t *config)
{
    return config != NULL &&
           config->kpMilliAPerRad <= MOTOR_IMPEDANCE_KP_MAX_MILLI_A_PER_RAD &&
           config->kdMilliASPerRad <= MOTOR_IMPEDANCE_KD_MAX_MILLI_A_S_PER_RAD &&
           config->transitionMs <= 1000U && config->feedforwardTransitionMs <= 1000U &&
           config->feedforwardIqMa >= -32767 && config->feedforwardIqMa <= 32767;
}

bool MotorImpedance_ConfigActive(const motor_impedance_config_t *config)
{
    return MotorImpedance_ConfigValid(config) &&
           (config->kpMilliAPerRad != 0U || config->kdMilliASPerRad != 0U);
}

void MotorImpedance_Clear(motor_impedance_state_t *state)
{
    if (state != NULL) memset(state, 0, sizeof(*state));
}

bool MotorImpedance_Reset(motor_impedance_state_t *state, float measuredVelocityRadS)
{
    if (state == NULL || !isfinite(measuredVelocityRadS)) return false;
    state->velocityFiltered = measuredVelocityRadS;
    state->filterInitialized = 1U;
    state->gainsInitialized = 0U;
    state->gainStepsLeft = 0U;
    state->feedforwardInitialized = 0U;
    state->feedforwardStepsLeft = 0U;
    state->feedforwardAppliedMa = 0.0f;
    return true;
}

bool MotorImpedance_SetTarget(motor_impedance_state_t *state, int32_t targetQ, uint16_t sequence)
{
    double radians;
    double counts;
    int64_t wholeCounts;
    if (state == NULL) return false;
    radians = (double)targetQ * 0.0001;
    counts = radians / MOTOR_IMPEDANCE_RAD_PER_COUNT;
    if (!isfinite(radians) || !isfinite(counts) ||
        counts > (double)INT64_MAX || counts < (double)INT64_MIN) {
        state->targetValid = 0U;
        return false;
    }
    wholeCounts = (int64_t)counts;
    state->targetCounts = wholeCounts;
    state->targetRemainder = (float)(radians - (double)wholeCounts * MOTOR_IMPEDANCE_RAD_PER_COUNT);
    state->targetSequence = sequence;
    state->targetValid = 1U;
    return isfinite(state->targetRemainder);
}

static int32_t diagnostic_scaled(float value, float scale, uint8_t *flags)
{
    double scaled = (double)value * (double)scale;
    if (!isfinite(scaled) || scaled > (double)INT32_MAX) {
        *flags |= MOTOR_IMPEDANCE_FLAG_DIAG_OVERFLOW;
        return INT32_MAX;
    }
    if (scaled < (double)INT32_MIN) {
        *flags |= MOTOR_IMPEDANCE_FLAG_DIAG_OVERFLOW;
        return INT32_MIN;
    }
    return (int32_t)lround(scaled);
}

motor_impedance_result_t MotorImpedance_Step(
    motor_impedance_state_t *state, const motor_impedance_config_t *config,
    int64_t measuredCounts, float measuredVelocityRadS, int8_t currentDirection,
    uint32_t currentLimitMa, motor_impedance_output_t *output)
{
    float error, pMa, dMa, rawAxisMa, rawIqMa, limitMa, commandMa;
    int64_t countError;
    if (output == NULL) return MOTOR_IMPEDANCE_BAD_INPUT;
    memset(output, 0, sizeof(*output));
    if (!MotorImpedance_ConfigActive(config)) {
        output->failure = MOTOR_IMPEDANCE_BAD_CONFIG;
        return MOTOR_IMPEDANCE_BAD_CONFIG;
    }
    if (state == NULL || !state->targetValid || !isfinite(state->targetRemainder)) {
        output->failure = MOTOR_IMPEDANCE_BAD_TARGET;
        return MOTOR_IMPEDANCE_BAD_TARGET;
    }
    if (!isfinite(measuredVelocityRadS)) {
        output->failure = MOTOR_IMPEDANCE_BAD_INPUT;
        return MOTOR_IMPEDANCE_BAD_INPUT;
    }
    if (currentDirection != 1 && currentDirection != -1) {
        output->failure = MOTOR_IMPEDANCE_BAD_DIRECTION;
        return MOTOR_IMPEDANCE_BAD_DIRECTION;
    }
    if (currentLimitMa == 0U) {
        output->failure = MOTOR_IMPEDANCE_BAD_LIMIT;
        return MOTOR_IMPEDANCE_BAD_LIMIT;
    }

    if (!state->filterInitialized) {
        state->velocityFiltered = measuredVelocityRadS;
        state->filterInitialized = 1U;
    } else {
        state->velocityFiltered += MOTOR_IMPEDANCE_SPEED_LPF_ALPHA *
                                   (measuredVelocityRadS - state->velocityFiltered);
    }
    if (!isfinite(state->velocityFiltered)) {
        output->failure = MOTOR_IMPEDANCE_BAD_INPUT;
        return MOTOR_IMPEDANCE_BAD_INPUT;
    }

    countError = state->targetCounts - measuredCounts;
    error = (float)countError * (float)MOTOR_IMPEDANCE_RAD_PER_COUNT + state->targetRemainder;
    /* Repeated target frames do not restart the ramp. Retarget from the
     * currently applied pair; no filter/position/lease state is reset. */
    if (!state->gainsInitialized || state->requestedKp != config->kpMilliAPerRad ||
        state->requestedKd != config->kdMilliASPerRad ||
        state->requestedTransitionMs != config->transitionMs) {
        state->requestedKp = config->kpMilliAPerRad;
        state->requestedKd = config->kdMilliASPerRad;
        state->requestedTransitionMs = config->transitionMs;
        state->gainStepsLeft = state->gainsInitialized ? config->transitionMs : 0U;
        state->gainsInitialized = 1U;
        if (state->gainStepsLeft) {
            state->kpStep = ((float)state->requestedKp - state->kpApplied) / state->gainStepsLeft;
            state->kdStep = ((float)state->requestedKd - state->kdApplied) / state->gainStepsLeft;
        }
    }
    if (state->gainStepsLeft) {
        state->kpApplied += state->kpStep;
        state->kdApplied += state->kdStep;
        state->gainStepsLeft--;
    }
    if (!state->gainStepsLeft) {
        state->kpApplied = (float)state->requestedKp;
        state->kdApplied = (float)state->requestedKd;
    }
    pMa = state->kpApplied * error;
    dMa = state->kdApplied * -state->velocityFiltered;
    rawAxisMa = pMa + dMa;
    /* Independent electrical-current ramp. A fresh ARM starts from zero;
     * gain-only updates must neither clear nor restart feedforward. */
    if (!state->feedforwardInitialized || state->requestedFeedforwardMa != config->feedforwardIqMa ||
        state->requestedFeedforwardMs != config->feedforwardTransitionMs) {
        state->requestedFeedforwardMa = config->feedforwardIqMa;
        state->requestedFeedforwardMs = config->feedforwardTransitionMs;
        state->feedforwardStepsLeft = config->feedforwardTransitionMs;
        state->feedforwardInitialized = 1U;
        if (state->feedforwardStepsLeft)
            state->feedforwardStep = ((float)state->requestedFeedforwardMa - state->feedforwardAppliedMa) /
                                     state->feedforwardStepsLeft;
    }
    if (state->feedforwardStepsLeft) {
        state->feedforwardAppliedMa += state->feedforwardStep;
        state->feedforwardStepsLeft--;
    }
    if (!state->feedforwardStepsLeft) state->feedforwardAppliedMa = (float)state->requestedFeedforwardMa;
    rawIqMa = (float)currentDirection * rawAxisMa + state->feedforwardAppliedMa;
    limitMa = (float)currentLimitMa;
    if (!isfinite(error) || !isfinite(pMa) || !isfinite(dMa) ||
        !isfinite(rawAxisMa) || !isfinite(rawIqMa) || !isfinite(limitMa)) {
        output->failure = MOTOR_IMPEDANCE_BAD_INPUT;
        return MOTOR_IMPEDANCE_BAD_INPUT;
    }

    commandMa = fmaxf(-limitMa, fminf(limitMa, rawIqMa));
    output->positionErrorRad = error;
    output->filteredVelocityRadS = state->velocityFiltered;
    output->positionErrorQ = diagnostic_scaled(error, 10000.0f, &output->flags);
    output->filteredVelocityMilliRadS = diagnostic_scaled(state->velocityFiltered, 1000.0f, &output->flags);
    output->pCurrentMa = diagnostic_scaled(pMa, 1.0f, &output->flags);
    output->dCurrentMa = diagnostic_scaled(dMa, 1.0f, &output->flags);
    output->rawAxisCurrentMa = diagnostic_scaled(rawAxisMa, 1.0f, &output->flags);
    output->feedforwardIqMa = diagnostic_scaled(state->feedforwardAppliedMa, 1.0f, &output->flags);
    output->rawElectricalMa = diagnostic_scaled(rawIqMa, 1.0f, &output->flags);
    output->commandIqMa = diagnostic_scaled(commandMa, 1.0f, &output->flags);
    output->iqCommandA = commandMa * 0.001f;
    output->targetSequence = state->targetSequence;
    output->flags |= MOTOR_IMPEDANCE_FLAG_VALID;
    if (rawIqMa > limitMa || rawIqMa < -limitMa)
        output->flags |= MOTOR_IMPEDANCE_FLAG_CLAMPED;
    if (!isfinite(output->iqCommandA)) {
        memset(output, 0, sizeof(*output));
        output->failure = MOTOR_IMPEDANCE_BAD_INPUT;
        return MOTOR_IMPEDANCE_BAD_INPUT;
    }
    return MOTOR_IMPEDANCE_OK;
}
