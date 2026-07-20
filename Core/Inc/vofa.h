/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ JustFloat 协议 — UART2 DMA 遥测发送 + 命令接收
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
#define VOFA_MAX_CHANNELS   14u
#define VOFA_TX_BUF_SIZE    384u
#define VOFA_RX_BUF_SIZE    256u

/* ── TX: 发送遥测数据 ────────────────────────────────────────────── */

/**
  * @brief  通过 UART2 DMA 以 VOFA+ JustFloat 格式发送浮点数据
  * @param  data   浮点数组
  * @param  count  通道数 (0..VOFA_MAX_CHANNELS)
  *
  * 输出格式: "channels: f1,f2,...,fN\n"，每通道保留 6 位小数
  */
void VOFA_SendData(const float *data, uint8_t count);

/* ── RX: 接收控制命令 ───────────────────────────────────────── */

/**
  * @brief  启动 UART2 DMA 接收（IDLE 线路检测）
  *         需在所有外设初始化后调用一次
  */
void VOFA_InitRx(void);

/**
  * @brief  处理接收到的命令帧
  *         在 main() 循环中周期性调用（~10-100 Hz 即可）
  */
void VOFA_ProcessCmd(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
