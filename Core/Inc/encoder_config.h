#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#define APP_ENCODER_AS5047P 1
#define APP_ENCODER_MT6835 2
#ifndef APP_ENCODER_TYPE
#define APP_ENCODER_TYPE APP_ENCODER_MT6835
#endif
#if APP_ENCODER_TYPE == APP_ENCODER_AS5047P
#define ENCODER_NATIVE_COUNTS 16384UL
#define ENCODER_NATIVE_TO_WIRE_SHIFT 0U
#define ENCODER_CAL_ANGLE_MODE (APP_ENCODER_RAW_ANGLE ? 1U : 2U)
#elif APP_ENCODER_TYPE == APP_ENCODER_MT6835
#define ENCODER_NATIVE_COUNTS 2097152UL
#define ENCODER_NATIVE_TO_WIRE_SHIFT 7U
/* Mode 4 identifies reversed MT6835 native21, matching the AS5047P convention.
 * Mode 3 was the old, unmodified MT6835 angle; never reuse that saved zero.
 * Record counts and CAN raw retain their 16384-count representation. */
#define ENCODER_CAL_ANGLE_MODE 4U
#else
#error "APP_ENCODER_TYPE must select AS5047P (1) or MT6835 (2)"
#endif

#endif
