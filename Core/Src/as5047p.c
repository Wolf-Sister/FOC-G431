/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    as5047p.c
  * @brief   AS5047P 14-bit Magnetic Rotary Position Sensor Driver (DMA Enabled)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "as5047p.h"

/* External SPI handle -------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* DMA 传输缓冲区（必须使用 16 位宽，确保存储对齐） -----------------------------*/
static uint16_t spi_tx_buf = 0x7FFEU; /* 固定的 ANGLEUNC 流水线读命令 */
static volatile uint16_t spi_rx_buf = 0x0000U; /* 接收缓冲区 */
static volatile uint8_t dma_transfer_busy = 0;

/* CS macro ------------------------------------------------------------------*/
#define CS_LOW()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

/* Private helpers -----------------------------------------------------------*/

static uint8_t AS5047P_CalcEvenParity(uint16_t data)
{
  uint16_t result = 0;
  while (data != 0)
  {
    result ^= data;
    data >>= 1;
  }
  return (uint8_t)(result & 0x01U);
}

static uint16_t AS5047P_BuildReadCmd(uint16_t reg_addr)
{
  uint16_t cmd = reg_addr & 0x3FFFU;
  if (AS5047P_CalcEvenParity(cmd) == 0U)
  {
    cmd |= 0x8000U;   
  }
  cmd |= 0x4000U;     
  return cmd;
}

/**
  * @brief  标准阻塞式 16-bit SPI 单帧传输 (用于初始化和普通非实时寄存器读取)
  */
static HAL_StatusTypeDef AS5047P_SPI_FrameTransfer(uint16_t tx_data, uint16_t *rx_data)
{
  HAL_StatusTypeDef status;
  *rx_data = 0;
  
  CS_LOW();
  for(volatile uint8_t i = 0; i < 15; i++); /* 满足 t_LCLK 建立时间 */
  
  /* 注意这里强制转换为 (uint16_t *)，触发 STM32 硬件 16 字节 FIFO 读写 */
  status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx_data, (uint8_t *)rx_data, 1, 10);
  
  for(volatile uint8_t i = 0; i < 5; i++);
  CS_HIGH();
  
  for(volatile uint8_t i = 0; i < 15; i++); /* 满足 t_CSH 释放延迟 */
  return status;
}

/* Public API ----------------------------------------------------------------*/

void AS5047P_Init(void)
{
  uint16_t rx;
  CS_HIGH();
  HAL_Delay(10); /* 充裕的上电时间 */

  /* 盲发 NOP 刷新硬件状态机 */
  AS5047P_SPI_FrameTransfer(0x0000U, &rx);
  AS5047P_SPI_FrameTransfer(0x0000U, &rx);
  AS5047P_SPI_FrameTransfer(0x0000U, &rx);
  
  dma_transfer_busy = 0;
}

uint8_t AS5047P_CheckParity(uint16_t data)
{
  return (AS5047P_CalcEvenParity(data) == 0U) ? 1U : 0U;
}

uint16_t AS5047P_ReadRegister(uint16_t reg_addr)
{
  if (dma_transfer_busy) return 0xFFFFU; /* 如果DMA正在占用SPI，拒绝离散读取 */
  
  uint16_t cmd = AS5047P_BuildReadCmd(reg_addr);
  uint16_t rx;

  AS5047P_SPI_FrameTransfer(cmd, &rx);
  AS5047P_SPI_FrameTransfer(0x0000U, &rx);

  if ((rx & 0x4000U) || !AS5047P_CheckParity(rx))
  {
    return 0xFFFFU;
  }
  return rx;
}

/* ========================== 高性能 DMA 核心流 ========================== */

/**
  * @brief  【异步非阻塞】异步启动一次 DMA 角度采集请求
  * @note   通常在 TIM1 更新中断(FOC环路起点)或者 ADC 转换完成中断中调用。
  * 该函数调用后立即返回，不占用 CPU 时间。
  */
void AS5047P_DMA_StartRequest(void)
{
  /* Critical section: mask IRQs at priority >= 1 (TIM2, DMA_CH1/CH2, SPI1),
   * but keep priority 0 (TIM1_UP, ADC1_2) unmasked so FOC is never blocked.
   *
   * NVIC priority 1 → BASEPRI[7:4]=0x1 → register value 0x10.
   * BASEPRI masks all IRQs with priority value >= 1 (larger number = lower priority).
   * Priority 0 (smallest number = highest priority) remains unmasked. */
  uint32_t basepri = __get_BASEPRI();
  __set_BASEPRI(0x10U);

  if (!dma_transfer_busy) {
    dma_transfer_busy = 1;
    CS_LOW();

    /* Start DMA transfer — DMA CH1 complete ISR at priority 1 is masked,
     * so it cannot clear dma_transfer_busy until BASEPRI is restored. */
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&spi_tx_buf,
                                (uint8_t *)&spi_rx_buf, 1);
  }

  __set_BASEPRI(basepri);  /* Restore previous BASEPRI */
}

/**
  * @brief  【DMA 结束回调】SPI DMA 传输完成中断回调函数
  * @note   此函数属于重写 HAL 库的 weak 虚函数。当 DMA 接收完数据后自动进入。
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    CS_HIGH();              /* 搬运完成，立刻拉高片选 */
    dma_transfer_busy = 0;  /* 释放忙标志 */
  }
}

/**
  * @brief  从 DMA 缓冲区中安全提取并校验上一次采集到的角度值
  * @retval 14位未补偿原始角度 (0-16383)，或 0xFFFF 报错
  */
uint16_t AS5047P_DMA_GetAngleCallback(void)
{
  uint16_t rx = spi_rx_buf;

  /* 校验奇偶校验位以及位 14 的 Error Flag */
  if ((rx & 0x4000U) || !AS5047P_CheckParity(rx))
  {
    return 0xFFFFU;
  }

  return rx & 0x3FFFU;
}

/* ===================================================================== */

uint16_t AS5047P_ReadAngleRaw(void)
{
  uint16_t rx = AS5047P_ReadRegister(AS5047P_REG_ANGLEUNC);
  if (rx == 0xFFFFU) return 0xFFFFU;
  return rx & 0x3FFFU;
}

uint16_t AS5047P_ReadAngleCompensated(void)
{
  uint16_t rx = AS5047P_ReadRegister(AS5047P_REG_ANGLECOM);
  if (rx == 0xFFFFU) return 0xFFFFU;
  return rx & 0x3FFFU;
}

float AS5047P_ReadAngleDegrees(void)
{
  uint16_t raw = AS5047P_ReadAngleRaw();
  if (raw == 0xFFFFU) return -1.0f;
  return (float)raw * (360.0f / 16384.0f);
}

uint16_t AS5047P_ReadMagnitude(void)
{
  uint16_t rx = AS5047P_ReadRegister(AS5047P_REG_MAG);
  if (rx == 0xFFFFU) return 0xFFFFU;
  return rx & 0x3FFFU;
}

uint8_t AS5047P_ReadErrorFlags(void)
{
  uint16_t rx = AS5047P_ReadRegister(AS5047P_REG_ERRFL);
  if (rx == 0xFFFFU) return 0xFFU;
  return (uint8_t)(rx & 0xFFU);
}
