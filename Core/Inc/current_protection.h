#ifndef CURRENT_PROTECTION_H
#define CURRENT_PROTECTION_H

#include <stdint.h>

/* Software current limits; validate analogue scaling and power-stage temperature
 * before treating them as a motor or board rating. */
#define CURRENT_REF_HARD_MAX_A       2.50f
#define CURRENT_SOFT_TRIP_A          2.80f
#define CURRENT_HARD_TRIP_A          3.00f
#define CURRENT_SOFT_TRIP_FRAMES     3U
#define CURRENT_ADC_MIN_CODE         32U
#define CURRENT_ADC_MAX_CODE         4063U
#define CURRENT_OFFSET_MIN_CODE      1920.0f
#define CURRENT_OFFSET_MAX_CODE      2175.0f
#define CURRENT_CAL_MAX_VARIANCE     64U
#define CURRENT_CAL_MAX_SPREAD       32U
#define CURRENT_CAL_HEADROOM_A       3.00f
#define CURRENT_CAL_SAMPLES          2000U
#define CURRENT_CAL_TIMEOUT_MS       200U
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
    uint64_t square_sum_a, square_sum_c;
    uint16_t count;
    uint16_t min_a, max_a, min_c, max_c;
    uint8_t soft_count[3];
    uint8_t calibrated, soft_fault;
    float offset_a, offset_c;
    float ia, ib, ic;
} current_protection_t;

void current_protection_reset(current_protection_t *state);
current_sample_result_t current_protection_sample(current_protection_t *state,
                                                 uint16_t raw_a, uint16_t raw_c);
#endif
