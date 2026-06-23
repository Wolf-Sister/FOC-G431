/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    as5047p.h
  * @brief   AS5047P 14-bit Magnetic Rotary Position Sensor Driver
  * @note    SPI1: PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI (Mode 0: CPOL=0, CPHA=0)
  *
  *          Command frame (16-bit, MSB first):
  *            Bit 15  : Parity (even parity over bits [14:0])
  *            Bit 14  : R/W (1 = read, 0 = write)
  *            Bits [13:0] : Register address
  *
  *          Response frame:
  *            Bit 15  : Parity (even parity over bits [14:0])
  *            Bit 14  : Error flag (EF, 0 = OK, 1 = error)
  *            Bits [13:0] : Register data (14-bit)
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

/* AS5047P Register Addresses (14-bit) ---------------------------------------*/
#define AS5047P_REG_NOP         0x0000U  /* No operation                        */
#define AS5047P_REG_ERRFL       0x0001U  /* Error flags                         */
#define AS5047P_REG_PROG        0x0003U  /* Programming control                 */
#define AS5047P_REG_DIAAGC      0x3FFCU  /* Diagnostics & AGC                   */
#define AS5047P_REG_MAG         0x3FFDU  /* CORDIC magnitude                    */
#define AS5047P_REG_ANGLEUNC    0x3FFEU  /* Angle, uncompensated                */
#define AS5047P_REG_ANGLECOM    0x3FFFU  /* Angle, with DAEC compensation        */

/* Non-Volatile Registers ----------------------------------------------------*/
#define AS5047P_REG_ZPOSM       0x0016U  /* Zero position (MSB)                 */
#define AS5047P_REG_ZPOSL       0x0017U  /* Zero position (LSB)                 */
#define AS5047P_REG_SETTINGS1   0x0018U  /* Settings 1 (ABI, resolution, etc.)   */
#define AS5047P_REG_SETTINGS2   0x0019U  /* Settings 2 (hysteresis, UVW, etc.)   */

/* Error flag bits (ERRFL register [7:0]) ------------------------------------*/
#define AS5047P_ERRFL_PARITY    0x01U  /* Parity error                        */
#define AS5047P_ERRFL_INVCOMM   0x02U  /* Invalid command                     */
#define AS5047P_ERRFL_FRAMING   0x08U  /* SPI framing error                   */
#define AS5047P_ERRFL_MAGL      0x10U  /* Magnetic field too low              */
#define AS5047P_ERRFL_MAGH      0x20U  /* Magnetic field too high             */
#define AS5047P_ERRFL_OFFSET    0x40U  /* Offset compensation not finished    */
#define AS5047P_ERRFL_WATCHDOG  0x80U  /* Logic watchdog timer expired        */

/* Angle conversion ----------------------------------------------------------*/
#define AS5047P_ANGLE_MAX       16383.0f  /* 14-bit max (2^14 - 1)              */

/* Function prototypes -------------------------------------------------------*/
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
void     AS5047P_DMA_StartRequest(void);
uint16_t AS5047P_DMA_GetAngleCallback(void);
void     AS5047P_ConfigUVWPolePairs(uint8_t pp);

#ifdef __cplusplus
}
#endif

#endif /* __AS5047P_H__ */
