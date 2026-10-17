#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "motor_profile.h"

/* Electrical angle polarity relative to the raw encoder, not joint coordinates.
 * This bench motor: positive electrical Iq rotates toward decreasing raw angle.
 * Also used by the speed PI to preserve negative feedback. */
#ifndef APP_MOTOR_ELECTRICAL_DIRECTION
#define APP_MOTOR_ELECTRICAL_DIRECTION (-1)
#endif
#if (APP_MOTOR_ELECTRICAL_DIRECTION != -1) && (APP_MOTOR_ELECTRICAL_DIRECTION != 1)
#error "APP_MOTOR_ELECTRICAL_DIRECTION must be -1 or 1"
#endif

/* UART2 command reception, telemetry and logs: 1=on, 0=off.
 * Default off for CAN operation; override to 1 for encoder bench commands.
 * A compiler definition may override this default for all source files. */
#ifndef APP_UART_ENABLE
#define APP_UART_ENABLE 0
#endif

#if (APP_UART_ENABLE != 0) && (APP_UART_ENABLE != 1)
#error "APP_UART_ENABLE must be 0 or 1"
#endif

/* CAN node service: disabling CAN never restores automatic closed-loop startup. */
#ifndef APP_CAN_ENABLE
#define APP_CAN_ENABLE 1
#endif
#if (APP_CAN_ENABLE != 0) && (APP_CAN_ENABLE != 1)
#error "APP_CAN_ENABLE must be 0 or 1"
#endif

/* Node identity and role are selected together in the compile-time profile. */
#define APP_CAN_NODE_ID MOTOR_PROFILE_NODE_ID
#define APP_MOTOR_ROLE MOTOR_PROFILE_ROLE
#if (APP_CAN_NODE_ID < 0) || (APP_CAN_NODE_ID > 63U)
#error "APP_CAN_NODE_ID must be 0..63"
#endif

#ifndef APP_FIRMWARE_VERSION
#define APP_FIRMWARE_VERSION 0x00010406U
#endif

/* Bump on wiring, encoder installation or electrical-angle semantics changes. */
#ifndef APP_ELECTRICAL_CAL_CONFIG_REV
#define APP_ELECTRICAL_CAL_CONFIG_REV 1U
#endif

/* Measured lower static noise: select CORDICANG at SPI 0x3FFF on every boot.
 * Set 0 to restore compensated-angle startup; no OTP programming. */
#define APP_ENCODER_RAW_ANGLE MOTOR_PROFILE_ENCODER_RAW_ANGLE
#ifndef APP_AUTO_CALIBRATE
/* Match the tested CAN workflow: explicit electrical alignment, then manual ARM. */
#define APP_AUTO_CALIBRATE 0U
#endif
#if (APP_AUTO_CALIBRATE != 0) && (APP_AUTO_CALIBRATE != 1)
#error "APP_AUTO_CALIBRATE must be 0 or 1"
#endif

/* Legacy UART-only timeout. Stage-two CAN always uses a 20 ms received-target
 * lease, independently of this setting; UART HB never renews CAN authority. */
#ifndef APP_COMMAND_TIMEOUT_MS
#define APP_COMMAND_TIMEOUT_MS 0U
#endif
#if APP_COMMAND_TIMEOUT_MS > 0x7FFFFFFFU
#error "APP_COMMAND_TIMEOUT_MS exceeds the supported wrap-safe interval"
#endif

#endif
