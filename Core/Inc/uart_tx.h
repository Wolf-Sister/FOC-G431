/**
  ******************************************************************************
  * @file    uart_tx.h
  * @brief   UART2 发送环形缓冲 — 单生产者写入，主循环统一泵出
  *
  *          设计要点:
  *            - UART_TX_Put() / UART_TX_PutString() 只做内存拷贝，绝不调用
  *              HAL；生产者调用不得相互抢占，运行期日志统一在主循环输出
  *            - 仅 UART_TX_Pump() 触碰 HAL 与 DMA，且只允许在主循环中调用
  *            - 缓冲满时整帧丢弃并计数，不写出半帧，也不阻塞
  *
  *          替代原先 UART2_SendString() 的 while 等待 + 直接 DMA 组合，
  *          该组合存在两个缺陷: 无超时死循环、以及把调用者栈缓冲交给 DMA。
  ******************************************************************************
  */

#ifndef __UART_TX_H__
#define __UART_TX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
/* 环形缓冲容量: 必须大于单帧最大长度 (VOFA 14 通道最坏约 178 B)             */
#define UART_TX_RING_SIZE     512U
/* 单次 DMA 传输上限: 决定线性暂存区大小                                     */
#define UART_TX_CHUNK_MAX     256U

/* 编译期保护: 单帧必须放得进线性暂存区，否则永不发送                        */
#if (UART_TX_CHUNK_MAX < 200U)
#error "UART_TX_CHUNK_MAX too small for the VOFA telemetry frame"
#endif

/* API ----------------------------------------------------------------------*/
void     UART_TX_Init(void);

/**
  * @brief  写入一段字节到发送缓冲（单生产者，不调用 HAL）
  * @param  data  源数据指针
  * @param  len   字节数
  * @retval true 已入队; false 空间不足已丢弃
  */
bool     UART_TX_Put(const char *data, uint16_t len);

/**
  * @brief  写入一个以 '\0' 结尾的字符串（与其他生产者串行调用）
  * @param  str  源字符串指针
  * @retval true 已入队; false 空间不足或参数无效
  */
bool     UART_TX_PutString(const char *str);

/**
  * @brief  尽力泵出一段缓冲（仅主循环允许调用）
  * @note   非阻塞: UART 忙时立即返回，下一个周期重试
  */
void     UART_TX_Pump(void);

/* 诊断计数 --------------------------------------------------------------*/
uint32_t UART_TX_Dropped(void);   /* 因缓冲满而丢弃的帧数                     */
uint32_t UART_TX_Pending(void);   /* 当前待发送字节数                         */
bool     UART_TX_Busy(void);      /* 1 = 仍有缓冲或 DMA 在进行                */

#ifdef __cplusplus
}
#endif

#endif /* __UART_TX_H__ */
