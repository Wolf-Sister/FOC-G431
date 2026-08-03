/**
  ******************************************************************************
  * @file    ina219.h
  * @brief   INA219AIDCNR 母线电压/电流检测 (I2C1)
  *
  *          仅用外部分流电阻检测母线电压和母线电流，与相电流采样(FOC电流环)
  *          完全独立。母线电压经滤波后用于更新 motor_config.voltage_supply，
  *          使 FOC 调制比换算跟随真实母线电压。
  ******************************************************************************
  */

#ifndef __INA219_H__
#define __INA219_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ========================================================================== */
/*  寄存器地址                                                                 */
/* ========================================================================== */
#define INA219_REG_CONFIG      0x00u
#define INA219_REG_SHUNT       0x01u
#define INA219_REG_BUS         0x02u
#define INA219_REG_POWER       0x03u
#define INA219_REG_CURRENT     0x04u
#define INA219_REG_CALIB       0x05u

/* I2C 地址: A0=A1=GND → 7-bit 0x40 (HAL 用 8-bit 左移一位)                    */
#define INA219_I2C_ADDR        0x40u

/* CONFIG: 16V 量程 | 分流 ±40mV | 12bit ADC | 连续转换 (shunt+bus)            */
/*   位解算: BRNG=0(16V), PG=00(±40mV), BADC=0011(12bit), SADC=0011(12bit), MODE=111(Continuous) */
/*   修正说明: 原 0x0FC0 的低三位为 000，会导致器件进入 Power-Down 挂起状态   */
#define INA219_CONFIG_VALUE    0x019Fu

/* CALIB: Current_LSB = 100µA/bit → 电流寄存器满量程 ≈3.28A                   */
/*   CAL = 0.04096 / (Current_LSB × Rshunt) = 0.04096/(1e-4×0.01) = 40960      */
#define INA219_CALIB_VALUE     40960u

/* 换算系数                                                                   */
#define INA219_BUSV_LSB_V      0.004f    /* 母线电压 4mV/LSB                   */
#define INA219_CURRENT_LSB_A   0.0001f   /* 母线电流 100µA/LSB                 */

/* I2C 超时 (ms) — 器件不在位/总线挂死时不卡死主循环                          */
#define INA219_I2C_TIMEOUT     10u

/* ========================================================================== */
/*  全局变量 — main 循环 100ms 更新                                            */
/* ========================================================================== */
extern volatile float ina219_vbus;   /* 母线电压 (V)                          */
extern volatile float ina219_ibus;   /* 母线电流 (A)                          */

/* ========================================================================== */
/*  API                                                                       */
/* ========================================================================== */
HAL_StatusTypeDef INA219_Init(void);
HAL_StatusTypeDef INA219_ReadBusVoltage(float *voltage);
HAL_StatusTypeDef INA219_ReadCurrent(float *current);

#ifdef __cplusplus
}
#endif

#endif /* __INA219_H__ */
