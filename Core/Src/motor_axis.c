#include "motor_axis.h"

#include <stddef.h>

static int64_t round_divide_symmetric(int64_t numerator, int64_t denominator)
{
    if (numerator >= 0) return (numerator + denominator / 2) / denominator;
    return -((-numerator + denominator / 2) / denominator);
}

bool MotorAxis_ConfigValid(const motor_axis_config_t *config)
{
    int64_t width;

    if (config == NULL || !config->referencePresent ||
        (config->direction != 1 && config->direction != -1) ||
        config->referenceRaw >= MOTOR_AXIS_ENCODER_COUNTS ||
        config->minimumQ >= config->maximumQ ||
        config->referenceQ < config->minimumQ ||
        config->referenceQ > config->maximumQ) {
        return false;
    }

    width = (int64_t)config->maximumQ - config->minimumQ;
    return width > 0 && width < MOTOR_AXIS_TURN_Q;
}

void MotorAxis_ZeroWindowReset(motor_axis_zero_window_t *window)
{
    if (window != NULL) *window = (motor_axis_zero_window_t){0};
}

static void zero_window_start(motor_axis_zero_window_t *window,
                              uint32_t nowMs,
                              uint32_t sequence,
                              uint16_t raw)
{
    *window = (motor_axis_zero_window_t){0};
    window->active = 1U;
    window->startedMs = nowMs;
    window->lastSampleMs = nowMs;
    window->lastSequence = sequence;
    window->originRaw = raw;
    window->sampleCount = 1U;
}

void MotorAxis_ZeroWindowAdd(motor_axis_zero_window_t *window,
                             uint32_t nowMs,
                             uint32_t encoderSequence,
                             uint16_t raw,
                             bool sampleValid)
{
    int32_t delta;
    if (window == NULL) return;
    if (!sampleValid || raw >= MOTOR_AXIS_ENCODER_COUNTS) {
        MotorAxis_ZeroWindowReset(window);
        return;
    }
    if (!window->active) {
        zero_window_start(window, nowMs, encoderSequence, raw);
        return;
    }
    if (encoderSequence == window->lastSequence) return;
    if ((uint32_t)(nowMs - window->lastSampleMs) > 2U || window->sampleCount >= 1000U) {
        zero_window_start(window, nowMs, encoderSequence, raw);
        return;
    }

    delta = (int32_t)raw - window->originRaw;
    if (delta > (int32_t)(MOTOR_AXIS_ENCODER_COUNTS / 2U)) delta -= (int32_t)MOTOR_AXIS_ENCODER_COUNTS;
    else if (delta < -(int32_t)(MOTOR_AXIS_ENCODER_COUNTS / 2U)) delta += (int32_t)MOTOR_AXIS_ENCODER_COUNTS;
    if (delta - window->minDelta > 8 || window->maxDelta - delta > 8) {
        zero_window_start(window, nowMs, encoderSequence, raw);
        return;
    }

    if (delta < window->minDelta) window->minDelta = delta;
    if (delta > window->maxDelta) window->maxDelta = delta;
    window->sumDelta += delta;
    window->sampleCount++;
    window->lastSequence = encoderSequence;
    window->lastSampleMs = nowMs;
}

bool MotorAxis_ZeroWindowRead(const motor_axis_zero_window_t *window,
                              uint32_t nowMs,
                              uint16_t *meanRaw)
{
    int64_t meanDelta;
    int32_t raw;
    if (window == NULL || meanRaw == NULL || !window->active || window->sampleCount < 50U ||
        (uint32_t)(nowMs - window->startedMs) < 100U ||
        (uint32_t)(nowMs - window->lastSampleMs) > 2U ||
        window->maxDelta - window->minDelta > 8) return false;

    meanDelta = round_divide_symmetric(window->sumDelta, window->sampleCount);
    raw = window->originRaw + (int32_t)meanDelta;
    if (raw < 0) raw += (int32_t)MOTOR_AXIS_ENCODER_COUNTS;
    else if (raw >= (int32_t)MOTOR_AXIS_ENCODER_COUNTS) raw -= (int32_t)MOTOR_AXIS_ENCODER_COUNTS;
    *meanRaw = (uint16_t)raw;
    return true;
}

motor_axis_result_t MotorAxis_MapRaw(const motor_axis_config_t *config,
                                     uint16_t raw,
                                     int32_t *jointQ,
                                     int8_t *branch)
{
    int32_t rawDelta;
    int32_t baseBranch = 0;
    int64_t deltaQ;
    int64_t midpointQ;
    int64_t halfTurnQ = MOTOR_AXIS_TURN_Q / 2;
    int64_t selected;
    int64_t branchAdjustment;
    int64_t selectedBranch;

    if (!MotorAxis_ConfigValid(config)) return MOTOR_AXIS_INVALID_CONFIG;
    if (raw >= MOTOR_AXIS_ENCODER_COUNTS || jointQ == NULL) return MOTOR_AXIS_INVALID_RAW;

    midpointQ = ((int64_t)config->minimumQ + config->maximumQ) / 2;
    rawDelta = (int32_t)raw - (int32_t)config->referenceRaw;
    if (rawDelta > (int32_t)(MOTOR_AXIS_ENCODER_COUNTS / 2U)) {
        rawDelta -= (int32_t)MOTOR_AXIS_ENCODER_COUNTS;
        baseBranch = -1;
    } else if (rawDelta < -(int32_t)(MOTOR_AXIS_ENCODER_COUNTS / 2U)) {
        rawDelta += (int32_t)MOTOR_AXIS_ENCODER_COUNTS;
        baseBranch = 1;
    }
    deltaQ = round_divide_symmetric((int64_t)rawDelta * MOTOR_AXIS_TURN_Q,
                                    MOTOR_AXIS_ENCODER_COUNTS);
    selected = (int64_t)config->referenceQ + config->direction * deltaQ;
    branchAdjustment = round_divide_symmetric((midpointQ - selected) * config->direction,
                                               MOTOR_AXIS_TURN_Q);
    selected += config->direction * branchAdjustment * MOTOR_AXIS_TURN_Q;
    selectedBranch = baseBranch + branchAdjustment;

    /* Use the half-open one-turn map interval. At an exact half-turn tie,
     * keep its positive endpoint so every raw angle has one stable branch. */
    if (selected <= midpointQ - halfTurnQ) {
        selected += MOTOR_AXIS_TURN_Q;
        selectedBranch += config->direction;
    } else if (selected > midpointQ + halfTurnQ) {
        selected -= MOTOR_AXIS_TURN_Q;
        selectedBranch -= config->direction;
    }
    if (selected <= midpointQ - halfTurnQ || selected > midpointQ + halfTurnQ ||
        selected < INT32_MIN || selected > INT32_MAX ||
        selectedBranch < INT8_MIN || selectedBranch > INT8_MAX)
        return MOTOR_AXIS_AMBIGUOUS;

    *jointQ = (int32_t)selected;
    if (branch != NULL) *branch = (int8_t)selectedBranch;
    return selected >= config->minimumQ && selected <= config->maximumQ ?
           MOTOR_AXIS_OK : MOTOR_AXIS_OUTSIDE_RANGE;
}

int64_t MotorAxis_QToCounts(int32_t jointQ)
{
    const double countsPerQ = 16384.0 / (6.283185307179586476925286766559 * 10000.0);
    double counts = (double)jointQ * countsPerQ;

    if (counts >= 0.0) return (int64_t)(counts + 0.5);
    return (int64_t)(counts - 0.5);
}

bool MotorAxis_JointSpeedToRaw(int32_t direction, float jointSpeed, float *rawSpeed)
{
    if ((direction != 1 && direction != -1) || rawSpeed == NULL) return false;
    *rawSpeed = (float)direction * jointSpeed;
    return true;
}
