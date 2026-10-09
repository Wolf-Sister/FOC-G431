#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* UART2 command reception, telemetry and logs: 1=on, 0=off.
 * Keep enabled while the CAN control interface is not implemented.
 * A compiler definition may override this default for all source files. */
#ifndef APP_UART_ENABLE
#define APP_UART_ENABLE 1
#endif

#if (APP_UART_ENABLE != 0) && (APP_UART_ENABLE != 1)
#error "APP_UART_ENABLE must be 0 or 1"
#endif

/* 0: attended bench keeps the last target. Remote integrations must choose a
 * finite lease and send valid motion commands or HB=1 before it expires. */
#ifndef APP_COMMAND_TIMEOUT_MS
#define APP_COMMAND_TIMEOUT_MS 0U
#endif
#if APP_COMMAND_TIMEOUT_MS > 0x7FFFFFFFU
#error "APP_COMMAND_TIMEOUT_MS exceeds the supported wrap-safe interval"
#endif

#endif
