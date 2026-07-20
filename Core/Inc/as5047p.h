/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    as5047p.h
  * @brief   AS5047P 14 位磁旋转位置传感器驱动
  * @note    SPI1: PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI (模式1: CPOL=0, CPHA=1)
  *
  *          命令帧 (16位, MSB 优先):
  *            Bit 15  : 奇偶校验 (位[14:0] 的偶校验)
  *            Bit 14  : R/W (1 = 读, 0 = 写)
  *            Bits [13:0] : 寄存器地址
  *
  *          响应帧:
  *            Bit 15  : 奇偶校验 (位[14:0] 的偶校验)
  *            Bit 14  : 错误标志 (EF, 0 = OK, 1 = 错误)
  *            Bits [13:0] : 寄存器数据 (14位)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __AS5047P_H__
#define __AS5047P_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* AS5047P 寄存器地址 (14位) ---------------------------------------*/
#define AS5047P_REG_NOP         0x0000U  /* 空操作                              */
#define AS5047P_REG_ERRFL       0x0001U  /* 错误标志                             */
#define AS5047P_REG_PROG        0x0003U  /* 编程控制                             */
#define AS5047P_REG_DIAAGC      0x3FFCU  /* 诊断 & AGC                           */
#define AS5047P_REG_MAG         0x3FFDU  /* CORDIC 幅值                          */
#define AS5047P_REG_ANGLEUNC    0x3FFEU  /* 角度，未补偿                          */
#define AS5047P_REG_ANGLECOM    0x3FFFU  /* 角度，带 DAEC 动态补偿                 */

/* 非易失性寄存器 ----------------------------------------------------*/
#define AS5047P_REG_ZPOSM       0x0016U  /* 零位 (MSB)                           */
#define AS5047P_REG_ZPOSL       0x0017U  /* 零位 (LSB)                           */
#define AS5047P_REG_SETTINGS1   0x0018U  /* 设置1 (ABI, 分辨率等)                  */
#define AS5047P_REG_SETTINGS2   0x0019U  /* 设置2 (迟滞, UVW等)                   */

/* 错误标志位 (ERRFL 寄存器 [7:0]) ------------------------------------*/
#define AS5047P_ERRFL_PARITY    0x04U  /* 奇偶校验错误                           */
#define AS5047P_ERRFL_INVCOMM   0x02U  /* 无效命令                              */
#define AS5047P_ERRFL_FRAMING   0x01U  /* SPI 帧错误                            */

/* DIAAGC 诊断位 ---------------------------------------------------*/
#define AS5047P_DIAAGC_MAGL     0x0800U /* 磁场过低                              */
#define AS5047P_DIAAGC_MAGH     0x0400U /* 磁场过高                              */
#define AS5047P_DIAAGC_COF      0x0200U /* CORDIC 溢出                           */
#define AS5047P_DIAAGC_LF       0x0100U /* 偏移补偿完成                           */

/* 角度转换 ----------------------------------------------------------*/
#define AS5047P_ANGLE_STEPS     16384.0f  /* 每机械圈 14 位计数                    */

/* 函数原型 -------------------------------------------------------*/
void     AS5047P_Init(void);
uint16_t AS5047P_ReadRegister(uint16_t reg_addr);
uint8_t  AS5047P_CheckParity(uint16_t data);
uint16_t AS5047P_ReadAngleRaw(void);
uint16_t AS5047P_ReadAngleCompensated(void);
uint16_t AS5047P_ReadAnglePipeline(void);
float    AS5047P_ReadAngleDegrees(void);
uint16_t AS5047P_ReadMagnitude(void);
uint8_t  AS5047P_ReadErrorFlags(void);
/* 高性能 DMA 接口原型 -------------------------------------------------------*/
HAL_StatusTypeDef AS5047P_DMA_StartRequest(void);
uint16_t AS5047P_DMA_GetAngleCallback(uint32_t *sample_cycle);
void     AS5047P_ConfigUVWPolePairs(uint8_t pp);

#ifdef __cplusplus
}
#endif

#endif /* __AS5047P_H__ */
