/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ JustFloat protocol — telemetry TX + command RX via UART2 DMA
  ******************************************************************************
  */

#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define VOFA_MAX_CHANNELS   10u
#define VOFA_TX_BUF_SIZE    256u
#define VOFA_RX_BUF_SIZE    256u

/* ── TX: Send telemetry data ────────────────────────────────────────────── */

/**
  * @brief  Send float data in VOFA+ JustFloat format via UART2 DMA (TX).
  * @param  data   Float array.
  * @param  count  Number of channels (0..VOFA_MAX_CHANNELS).
  *
  * Output: "channels: f1,f2,...,fN\n" — 6 decimal places each.
  */
void VOFA_SendData(const float *data, uint8_t count);

/* ── RX: Receive control commands ───────────────────────────────────────── */

/**
  * @brief  Start UART2 DMA reception (IDLE-line detection).
  *         Must be called once after all peripherals are initialized.
  */
void VOFA_InitRx(void);

/**
  * @brief  Process any received command frame.
  *         Call periodically in main() loop (~10–100 Hz is fine).
  */
void VOFA_ProcessCmd(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
