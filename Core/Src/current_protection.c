#include "current_protection.h"
#include <math.h>
#include <string.h>

void current_protection_reset(current_protection_t *state)
{
    memset(state, 0, sizeof(*state));
    state->min_a = UINT16_MAX;
    state->min_c = UINT16_MAX;
}

static int current_offset_valid(float offset, uint16_t lo, uint16_t hi)
{
    return isfinite(offset) && offset >= CURRENT_OFFSET_MIN_CODE &&
           offset <= CURRENT_OFFSET_MAX_CODE &&
           (uint16_t)(hi - lo) <= CURRENT_CAL_MAX_SPREAD &&
           (offset - CURRENT_ADC_MIN_CODE) * CURRENT_AMPS_PER_CODE >= CURRENT_CAL_HEADROOM_A &&
           (CURRENT_ADC_MAX_CODE - offset) * CURRENT_AMPS_PER_CODE >= CURRENT_CAL_HEADROOM_A;
}

current_sample_result_t current_protection_sample(current_protection_t *state,
                                                 uint16_t raw_a, uint16_t raw_c)
{
    float phase[3];
    uint8_t i;
    if (raw_a < CURRENT_ADC_MIN_CODE || raw_a > CURRENT_ADC_MAX_CODE ||
        raw_c < CURRENT_ADC_MIN_CODE || raw_c > CURRENT_ADC_MAX_CODE) {
        return CURRENT_SAMPLE_ADC_RANGE;
    }
    if (!state->calibrated) {
        state->square_sum_a += (uint32_t)raw_a * raw_a;
        state->square_sum_c += (uint32_t)raw_c * raw_c;
        state->sum_a += raw_a;
        state->sum_c += raw_c;
        if (raw_a < state->min_a) state->min_a = raw_a;
        if (raw_a > state->max_a) state->max_a = raw_a;
        if (raw_c < state->min_c) state->min_c = raw_c;
        if (raw_c > state->max_c) state->max_c = raw_c;
        state->count++;
        if (state->count < CURRENT_CAL_SAMPLES) return CURRENT_SAMPLE_CALIBRATING;
        state->offset_a = (float)state->sum_a / CURRENT_CAL_SAMPLES;
        state->offset_c = (float)state->sum_c / CURRENT_CAL_SAMPLES;
        if (!current_offset_valid(state->offset_a, state->min_a, state->max_a) ||
            !current_offset_valid(state->offset_c, state->min_c, state->max_c)) {
            return CURRENT_SAMPLE_CAL_INVALID;
        }
        if (state->square_sum_a * CURRENT_CAL_SAMPLES - (uint64_t)state->sum_a * state->sum_a >
                (uint64_t)CURRENT_CAL_MAX_VARIANCE * CURRENT_CAL_SAMPLES * CURRENT_CAL_SAMPLES ||
            state->square_sum_c * CURRENT_CAL_SAMPLES - (uint64_t)state->sum_c * state->sum_c >
                (uint64_t)CURRENT_CAL_MAX_VARIANCE * CURRENT_CAL_SAMPLES * CURRENT_CAL_SAMPLES)
            return CURRENT_SAMPLE_CAL_INVALID;
        state->calibrated = 1U;
        return CURRENT_SAMPLE_CALIBRATED;
    }
    state->ia = ((float)raw_a - state->offset_a) * CURRENT_AMPS_PER_CODE;
    state->ic = ((float)raw_c - state->offset_c) * CURRENT_AMPS_PER_CODE;
    state->ib = -(state->ia + state->ic);
    phase[0] = state->ia;
    phase[1] = state->ib;
    phase[2] = state->ic;
    for (i = 0U; i < 3U; i++) {
        float magnitude = fabsf(phase[i]);
        if (!isfinite(magnitude)) return CURRENT_SAMPLE_NONFINITE;
        if (magnitude > CURRENT_HARD_TRIP_A) { state->soft_fault = 0U; return CURRENT_SAMPLE_OVERCURRENT; }
    }
    for (i = 0U; i < 3U; i++) {
        float magnitude = fabsf(phase[i]);
        if (magnitude > CURRENT_SOFT_TRIP_A) {
            if (state->soft_count[i] < CURRENT_SOFT_TRIP_FRAMES) state->soft_count[i]++;
            if (state->soft_count[i] >= CURRENT_SOFT_TRIP_FRAMES) { state->soft_fault = 1U; return CURRENT_SAMPLE_OVERCURRENT; }
        } else {
            state->soft_count[i] = 0U;
        }
    }
    return CURRENT_SAMPLE_VALID;
}
