/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    as5047p.c
  * @brief   AS5047P 14 位磁旋转位置传感器驱动 (DMA 模式)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "as5047p.h"
#include "cmsis_compiler.h"               /* __get_BASEPRI / __set_BASEPRI 临界区保护 */

/* 外部 SPI 句柄 -------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* DMA 传输缓冲区（必须使用 16 位宽，确保存储对齐） -----------------------------*/
static uint16_t spi_tx_buf = 0x0000U; /* 初始化时生成 ANGLECOM 读命令 */
static volatile uint16_t spi_rx_buf = 0x0000U; /* 接收缓冲区 */
static volatile uint8_t dma_transfer_busy = 0;
static volatile uint8_t dma_sample_ready = 0;
static volatile uint8_t dma_response_primed = 0;
static volatile uint32_t dma_request_cycle = 0U;
static volatile uint32_t dma_pipeline_cycle = 0U;
static volatile uint32_t dma_sample_cycle = 0U;

/* CS 片选宏 ------------------------------------------------------------------*/
#define CS_LOW()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

/* 私有辅助函数 -----------------------------------------------------------*/

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
  uint16_t cmd = (reg_addr & 0x3FFFU) | 0x4000U;
  if (AS5047P_CalcEvenParity(cmd) != 0U) {
    cmd |= 0x8000U;
  }
  return cmd;
}

static uint16_t AS5047P_BuildDataFrame(uint16_t data)
{
  uint16_t frame = data & 0x3FFFU;
  if (AS5047P_CalcEvenParity(frame) != 0U) {
    frame |= 0x8000U;
  }
  return frame;
}

static HAL_StatusTypeDef AS5047P_SPI_FrameTransfer(uint16_t tx_data, uint16_t *rx_data);

/**
  * @brief  写 AS5047P 寄存器（仅 volatile 写入，不触发 OTP 烧录）
  * @param  reg_addr  14-bit 寄存器地址
  * @param  data      14-bit 待写入数据
  * @retval HAL status
  */
static HAL_StatusTypeDef AS5047P_WriteRegister(uint16_t reg_addr, uint16_t data)
{
  if (dma_transfer_busy) return HAL_BUSY;

  uint16_t cmd = AS5047P_BuildDataFrame(reg_addr);
  uint16_t tx = AS5047P_BuildDataFrame(data);
  uint16_t rx;

  HAL_StatusTypeDef status = AS5047P_SPI_FrameTransfer(cmd, &rx);
  if (status != HAL_OK) return status;
  return AS5047P_SPI_FrameTransfer(tx, &rx);
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

/* 公开 API ----------------------------------------------------------------*/

void AS5047P_Init(void)
{
  uint16_t rx;
  CS_HIGH();
  HAL_Delay(10); /* 充裕的上电时间 */

  /* 盲发 NOP 刷新硬件状态机 */
  uint16_t nop = AS5047P_BuildReadCmd(AS5047P_REG_NOP);
  AS5047P_SPI_FrameTransfer(nop, &rx);
  AS5047P_SPI_FrameTransfer(nop, &rx);
  AS5047P_SPI_FrameTransfer(nop, &rx);

  /* 配置 UVW 极对数为 1（匹配 2 极辅助磁铁） */
  AS5047P_ConfigUVWPolePairs(1);

  dma_transfer_busy = 0;
  dma_sample_ready = 0;
  dma_response_primed = 0;
  dma_request_cycle = 0U;
  dma_pipeline_cycle = 0U;
  dma_sample_cycle = 0U;
  spi_tx_buf = AS5047P_BuildReadCmd(AS5047P_REG_ANGLECOM);
}

uint8_t AS5047P_CheckParity(uint16_t data)
{
  return (AS5047P_CalcEvenParity(data) == 0U) ? 1U : 0U;
}

uint16_t AS5047P_ReadRegister(uint16_t reg_addr)
{
  if (dma_transfer_busy) return 0xFFFFU; /* 如果DMA正在占用SPI，拒绝离散读取 */

  uint16_t cmd = AS5047P_BuildReadCmd(reg_addr);
  uint16_t nop = AS5047P_BuildReadCmd(AS5047P_REG_NOP);
  uint16_t rx;

  /* 每个 16-bit 帧都必须由独立的 CS 上升沿结束。 */
  HAL_StatusTypeDef status = AS5047P_SPI_FrameTransfer(cmd, &rx);
  if (status != HAL_OK) return 0xFFFFU;

  status = AS5047P_SPI_FrameTransfer(nop, &rx);
  if (status != HAL_OK || (rx & 0x4000U) || !AS5047P_CheckParity(rx)) {
    return 0xFFFFU;
  }
  return rx;
}

/**
  * @brief  配置 UVW 输出极对数（严格匹配官方手册 SETTINGS2[2:0] UVWPP） 
  * @param  pp  极对数支持可选: 1, 3, 4, 5, 7（其他值将被忽略） 
  * @note   仅写 volatile 影子寄存器，掉电丢失，需每次上电配置 [cite: 368, 369]
  */
void AS5047P_ConfigUVWPolePairs(uint8_t pp)
{
  uint8_t uvw_bits;
  
  /* 根据手册 Figure 29 严格对齐低 3 位的编码映射  */
  switch (pp) {
    case 1:  uvw_bits = 0x00U; break;  /* 000 = 1 pole pair    */
    case 3:  uvw_bits = 0x02U; break;  /* 010 = 3 pole pairs   */
    case 4:  uvw_bits = 0x03U; break;  /* 011 = 4 pole pairs   */
    case 5:  uvw_bits = 0x04U; break;  /* 100 = 5 pole pairs   */
    case 7:  uvw_bits = 0x06U; break;  /* 110 = 7 pole pairs   */
    default: return;                   /* 不支持的极对数直接退出 */
  }

  /* 1. 读取当前寄存器全值 */
  uint16_t reg = AS5047P_ReadRegister(AS5047P_REG_SETTINGS2);
  if (reg == 0xFFFFU) return;

  /* 2. 修改位域 */
  reg &= ~0x0007U;       /* 精准清空 UVWPP 位 [2:0]（即低 3 位）  */
  reg |= uvw_bits;       /* 写入正确的极对数编码  */

  /* 3. 写入寄存器（内部会自动通过 16-bit SPI 触发写后立即校验） */
  AS5047P_WriteRegister(AS5047P_REG_SETTINGS2, reg);
}

/* ========================== 高性能 DMA 核心流 ========================== */

/**
  * @brief  【异步非阻塞】异步启动一次 DMA 角度采集请求
  * @note   通常在 TIM1 更新中断(FOC环路起点)或者 ADC 转换完成中断中调用。
  * 该函数调用后立即返回，不占用 CPU 时间。
  */
HAL_StatusTypeDef AS5047P_DMA_StartRequest(void)
{
  /* 临界区：屏蔽优先级 >= 1 的中断 (TIM2, DMA_CH1/CH2, SPI1)，
   * 保持优先级 0 (TIM1_UP, ADC1_2) 不被屏蔽，确保 FOC 永不阻塞。
   *
   * NVIC 优先级 1 → BASEPRI[7:4]=0x1 → 寄存器值 0x10。
   * BASEPRI 屏蔽所有优先级数值 >= 1 的中断 (数值越大优先级越低)。
   * 优先级 0 (数值最小 = 最高优先级) 保持不被屏蔽。 */
  uint32_t basepri = __get_BASEPRI();
  __set_BASEPRI(0x10U);
  HAL_StatusTypeDef status = HAL_BUSY;

  if (!dma_transfer_busy) {
    dma_transfer_busy = 1;
    CS_LOW();
    dma_request_cycle = DWT->CYCCNT;

    status = HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&spi_tx_buf,
                                         (uint8_t *)&spi_rx_buf, 1);
    if (status != HAL_OK) {
      CS_HIGH();
      dma_transfer_busy = 0;
      dma_sample_ready = 0;
    }
  }

  __set_BASEPRI(basepri);
  return status;
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
    /* The response belongs to the previous SPI command. Advance the saved
     * request time only after associating that previous command with RX. */
    dma_sample_cycle = dma_pipeline_cycle;
    dma_pipeline_cycle = dma_request_cycle;
    dma_sample_ready = 1;   /* 接收缓冲区包含一个新的完整响应 */
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    CS_HIGH();
    dma_transfer_busy = 0;
    dma_sample_ready = 0;
    dma_response_primed = 0;
    dma_request_cycle = 0U;
    dma_pipeline_cycle = 0U;
    dma_sample_cycle = 0U;
  }
}

/**
  * @brief  从 DMA 缓冲区中安全提取并校验上一次采集到的角度值
  * @retval 14位未补偿原始角度 (0-16383)，或 0xFFFF 报错
  */
uint16_t AS5047P_DMA_GetAngleCallback(uint32_t *sample_cycle)
{
  if (sample_cycle != 0) {
    *sample_cycle = 0U;
  }

  uint32_t basepri = __get_BASEPRI();
  __set_BASEPRI(0x10U);
  if (!dma_sample_ready) {
    __set_BASEPRI(basepri);
    return 0xFFFFU;
  }

  uint16_t rx = spi_rx_buf;
  uint32_t captured_cycle = dma_sample_cycle;
  dma_sample_ready = 0;
  __set_BASEPRI(basepri);

  /* 首次 DMA 返回的是流水线启动前的残留响应，不能作为角度值使用 */
  if (!dma_response_primed) {
    dma_response_primed = 1;
    return 0xFFFFU;
  }
  /* 校验奇偶校验位以及位 14 的 Error Flag */
  if ((rx & 0x4000U) || !AS5047P_CheckParity(rx))
  {
    return 0xFFFFU;
  }

  if (sample_cycle != 0) {
    *sample_cycle = captured_cycle;
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
