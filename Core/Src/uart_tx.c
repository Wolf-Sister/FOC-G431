/**
  ******************************************************************************
  * @file    uart_tx.c
  * @brief   UART2 发送环形缓冲实现
  *
  *          生产者 (调用必须串行)            消费者 (仅主循环)
  *            UART_TX_Put()                    UART_TX_Pump()
  *              → 拷贝进环形缓冲                 → 搬入线性暂存区
  *              → 不触碰 HAL                     → HAL_UART_Transmit_DMA()
  *
  *          单生产者 + 单消费者模型，用"先写数据、后提交索引"的顺序保证
  *          读者永远不会看到半帧；__DMB() 保证写指针在数据之后对其他
  *          总线主设备可见。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "uart_tx.h"
#include "usart.h"
#include <string.h>
#include "vofa.h"

/* Private variables ---------------------------------------------------------*/
#if APP_UART_ENABLE
static uint8_t  tx_ring[UART_TX_RING_SIZE];
/* DMA 线性暂存区: 必须是静态存储，不能是栈变量。
 * 原 UART2_SendString() 直接把调用者的栈缓冲交给 DMA，函数返回后缓冲
 * 即被复用，属于真实的数据竞争缺陷。                                   */
static uint8_t  tx_stage[UART_TX_CHUNK_MAX];

static volatile uint16_t tx_write = 0U;   /* 生产者提交位置 (单调递增)         */
static volatile uint16_t tx_read  = 0U;   /* 消费者送出位置 (单调递增)         */
static volatile uint8_t  tx_dma_active = 0U;

static volatile uint32_t tx_dropped = 0U;

/* Private functions ---------------------------------------------------------*/

static uint16_t UART_TX_Used(void)
{
    return (uint16_t)(tx_write - tx_read);
}

/* Public functions ----------------------------------------------------------*/

void UART_TX_Init(void)
{
    tx_write = 0U;
    tx_read  = 0U;
    tx_dma_active = 0U;
    tx_dropped = 0U;
    (void)memset(tx_ring, 0, sizeof(tx_ring));
    (void)memset(tx_stage, 0, sizeof(tx_stage));
}

bool UART_TX_Put(const char *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return false;
    }
    /* 单帧超过线性暂存区则永远发不出去，直接拒绝而不是写坏缓冲 */
    if (len > UART_TX_CHUNK_MAX) {
        tx_dropped++;
        return false;
    }
    /* 满则整帧丢弃，避免写出半帧导致上位机协议解析错乱 */
    if ((uint16_t)(UART_TX_RING_SIZE - UART_TX_Used()) <= len) {
        tx_dropped++;
        return false;
    }

    for (uint16_t i = 0U; i < len; i++) {
        tx_ring[(uint16_t)((tx_write + i) % UART_TX_RING_SIZE)] = (uint8_t)data[i];
    }

    __DMB();                 /* 数据先可见，索引后提交 */
    tx_write = (uint16_t)(tx_write + len);
    return true;
}

bool UART_TX_PutString(const char *str)
{
    if (str == NULL) {
        return false;
    }
    size_t len = strlen(str);
    if (len == 0U) {
        return true;
    }
    if (len > 0xFFFFU) {
        return false;
    }
    return UART_TX_Put(str, (uint16_t)len);
}

void UART_TX_Pump(void)
{
    /* DMA 忙则立即返回，不阻塞。TxCplt 回调下一轮会再次进入。 */
    if (tx_dma_active != 0U || huart2.gState != HAL_UART_STATE_READY) {
        return;
    }
    if (tx_read == tx_write) {
        return;                                  /* 无待发数据 */
    }

    uint16_t avail = UART_TX_Used();
    uint16_t chunk = (avail > UART_TX_CHUNK_MAX) ? UART_TX_CHUNK_MAX : avail;

    /* 环形 → 线性: DMA 不能跨缓冲边界回绕 */
    for (uint16_t i = 0U; i < chunk; i++) {
        tx_stage[i] = tx_ring[(uint16_t)((tx_read + i) % UART_TX_RING_SIZE)];
    }
    __DMB();

    tx_dma_active = 1U;
    if (HAL_UART_Transmit_DMA(&huart2, tx_stage, chunk) == HAL_OK) {
        tx_read = (uint16_t)(tx_read + chunk);
    } else {
        /* 发送启动失败: 回退标志，下一轮重试，已搬出的数据不算丢失
         * (tx_read 未推进，暂存区内容下轮会被重新搬运) */
        tx_dma_active = 0U;
    }
}

uint32_t UART_TX_Dropped(void)
{
    return tx_dropped;
}

uint32_t UART_TX_Pending(void)
{
    return (uint32_t)UART_TX_Used();
}

bool UART_TX_Busy(void)
{
    return (tx_dma_active != 0U) || (tx_read != tx_write);
}

/* HAL callback --------------------------------------------------------------*/

/**
  * @brief  UART 错误回调
  * @note   RX错误不一定中止TX；只在HAL确认TX已结束时释放源缓冲。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        VOFA_NotifyRxError();
        if (huart->gState == HAL_UART_STATE_READY && huart->hdmatx != NULL &&
            (huart->hdmatx->Instance->CCR & DMA_CCR_EN) == 0U) {
            CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
            tx_dma_active = 0U;
        }
    }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) tx_dma_active = 0U;
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        VOFA_NotifyRxError();
        tx_dma_active = 0U;
    }
}

/**
  * @brief  UART DMA 发送完成回调
  * @note   只清标志，不做任何搬运；真正的续传由下一次 UART_TX_Pump() 完成。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        tx_dma_active = 0U;
    }
}
#else
void UART_TX_Init(void) {}
bool UART_TX_Put(const char *data, uint16_t len)
{
    (void)data;
    (void)len;
    return false;
}
bool UART_TX_PutString(const char *str)
{
    (void)str;
    return false;
}
void UART_TX_Pump(void) {}
uint32_t UART_TX_Dropped(void) { return 0U; }
uint32_t UART_TX_Pending(void) { return 0U; }
bool UART_TX_Busy(void) { return false; }
#endif
