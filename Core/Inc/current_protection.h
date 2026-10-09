#ifndef CURRENT_PROTECTION_H
#define CURRENT_PROTECTION_H

#include <stdint.h>

/* Initial bench configuration; verify against the motor and measured analogue
 * supply/noise before increasing current or using this as a board rating. */
#define CURRENT_REF_HARD_MAX_A       1.50f
#define CURRENT_SOFT_TRIP_A          1.80f
#define CURRENT_HARD_TRIP_A          2.40f
#define CURRENT_SOFT_TRIP_FRAMES     3U
#define CURRENT_ADC_MIN_CODE         249U
#define CURRENT_ADC_MAX_CODE         3723U
#define CURRENT_OFFSET_MIN_CODE      1800.0f
#define CURRENT_OFFSET_MAX_CODE      2296.0f
#define CURRENT_CAL_MAX_SPREAD       32U
#define CURRENT_CAL_HEADROOM_A       2.55f
#define CURRENT_CAL_SAMPLES          2000U
#define CURRENT_CAL_TIMEOUT_MS       500U
#define CURRENT_ADC_TIMEOUT_MS       2U
#define CURRENT_AMPS_PER_CODE        (3.3f / (4096.0f * 0.01f * 50.0f))

typedef enum {
    CURRENT_SAMPLE_CALIBRATING,
    CURRENT_SAMPLE_CALIBRATED,
    CURRENT_SAMPLE_VALID,
    CURRENT_SAMPLE_ADC_RANGE,
    CURRENT_SAMPLE_CAL_INVALID,
    CURRENT_SAMPLE_OVERCURRENT,
    CURRENT_SAMPLE_NONFINITE
} current_sample_result_t;

typedef struct {
    uint32_t sum_a, sum_c;
    uint16_t count;
    uint16_t min_a, max_a, min_c, max_c;
    uint8_t soft_count[3];
    uint8_t calibrated;
    float offset_a, offset_c;
    float ia, ib, ic;
} current_protection_t;

void current_protection_reset(current_protection_t *state);
current_sample_result_t current_protection_sample(current_protection_t *state,
                                                 uint16_t raw_a, uint16_t raw_c);
#endif
