#ifndef BOARD_SAFETY_PROFILE_H
#define BOARD_SAFETY_PROFILE_H

#include "current_protection.h"

/* User-requested bench current profile: 1.5 A initial, 2.5 A command ceiling,
 * 2.8 A soft trip and 3.0 A hard trip. These are not measured motor/board
 * ratings or installation qualification. CAN cannot write this profile. */
#ifndef BOARD_SAFETY_PROFILE_VALIDATED
#define BOARD_SAFETY_PROFILE_VALIDATED 1U
#endif
#if (BOARD_SAFETY_PROFILE_VALIDATED != 0) && (BOARD_SAFETY_PROFILE_VALIDATED != 1)
#error "BOARD_SAFETY_PROFILE_VALIDATED must be 0 or 1"
#endif

/* Bench current profile; software limits are not validated component ratings. */
#define BOARD_REF_MAX_MA 2500U
#define BOARD_INITIAL_IQ_LIMIT_MA 1500U
/* Motion defaults are configurable ceilings, not motor ratings or run targets. */
#define BOARD_DEFAULT_SPEED_LIMIT_MRAD_S 100000U
#define BOARD_DEFAULT_POSITION_SPEED_MRAD_S 100000U
#define BOARD_DEFAULT_ACCEL_MRAD_S2 25000U
#define BOARD_DEFAULT_POSITION_MIN  -327670000U
#define BOARD_DEFAULT_POSITION_MAX  327670000U
#define BOARD_SPEED_MAX_MRAD_S 150000U
#define BOARD_POSITION_SPEED_MAX_MRAD_S 100000U
#define BOARD_ACCEL_MAX_MRAD_S2 50000U
#define MOTOR_POSITION_ARM_MAX_ERROR_RAD 0.05f
#define MOTOR_TARGET_LEASE_MS 20U
#define MOTOR_TARGET_WARN_MS 10U
#define MOTOR_INITIAL_TARGET_MAX_AGE_MS 10U
#define MOTOR_CONTROL_STALL_MS 2U
#define MOTOR_MAIN_STALL_MS 20U

#if BOARD_INITIAL_IQ_LIMIT_MA > BOARD_REF_MAX_MA
#error "Initial current exceeds board reference maximum"
#endif

#endif
