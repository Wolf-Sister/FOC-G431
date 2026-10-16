#ifndef APP_CONFIG_H
#define APP_CONFIG_H

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

/* Explicit seed for this bench board. Assign a unique 1..63 to every board;
 * 0 is unconfigured and sends no node-addressed traffic. No Flash writes. */
#ifndef APP_CAN_NODE_ID
#define APP_CAN_NODE_ID 1U
#endif
#if (APP_CAN_NODE_ID < 0) || (APP_CAN_NODE_ID > 63U)
#error "APP_CAN_NODE_ID must be 0..63"
#endif

#ifndef APP_FIRMWARE_VERSION
#define APP_FIRMWARE_VERSION 0x00010306U
#endif

/* Measured lower static noise: select CORDICANG at SPI 0x3FFF on every boot.
 * Set 0 to restore compensated-angle startup; no OTP programming. */
#ifndef APP_ENCODER_RAW_ANGLE
#define APP_ENCODER_RAW_ANGLE 1
#endif
#if (APP_ENCODER_RAW_ANGLE != 0) && (APP_ENCODER_RAW_ANGLE != 1)
#error "APP_ENCODER_RAW_ANGLE must be 0 or 1"
#endif

/* This board is an uninstalled bench motor: continuous-axis (WHEEL) semantics.
 * Set 1 for an installed JOINT; ordinary JOINT ARM needs stage-three limits. */
#ifndef APP_MOTOR_ROLE
#define APP_MOTOR_ROLE 2U
#endif
#if (APP_MOTOR_ROLE < 0) || (APP_MOTOR_ROLE > 2U)
#error "APP_MOTOR_ROLE must be 0, 1 or 2"
#endif
#ifndef APP_AUTO_CALIBRATE
#define APP_AUTO_CALIBRATE 1U
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
