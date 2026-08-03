/**
  ******************************************************************************
  * @file    ina219.c
  * @brief   INA219AIDCNR 母线电压/电流检测 (I2C1) — 轮询读取，带超时防护
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ina219.h"
#include "i2c.h"        /* hi2c1 */
#include <stddef.h>     /* NULL */

/* ========================================================================== */
/*  全局变量                                                                   */
/* ========================================================================== */
volatile float ina219_vbus = 0.0f;
volatile float ina219_ibus = 0.0f;

/* ========================================================================== */
/*  底层寄存器读写                                                             */
/* ========================================================================== */
static HAL_StatusTypeDef INA219_ReadReg(uint8_t reg, uint16_t *value)
{
    if (value == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t buf[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(INA219_I2C_ADDR << 1),
                         reg, I2C_MEMADD_SIZE_8BIT, buf, 2,
                         INA219_I2C_TIMEOUT) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *value = (uint16_t)((buf[0] << 8) | buf[1]);
    return HAL_OK;
}

static HAL_StatusTypeDef INA219_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFFu);

    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(INA219_I2C_ADDR << 1),
                             reg, I2C_MEMADD_SIZE_8BIT, buf, 2,
                             INA219_I2C_TIMEOUT);
}

/* ========================================================================== */
/*  初始化: 验证器件在位 + 写 CONFIG/CALIB                                     */
/* ========================================================================== */
HAL_StatusTypeDef INA219_Init(void)
{
    uint16_t cfg = 0;

    /* 读 CONFIG 验证器件在位 (总线无应答读回 0xFFFF = 未接线/未焊上)          */
    if (INA219_ReadReg(INA219_REG_CONFIG, &cfg) != HAL_OK || cfg == 0xFFFFu)
    {
        return HAL_ERROR;
    }

    /* 写入工作配置寄存器 */
    if (INA219_WriteReg(INA219_REG_CONFIG, INA219_CONFIG_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 写入校准寄存器 */
    if (INA219_WriteReg(INA219_REG_CALIB, INA219_CALIB_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  读取母线电压 (V): Vbus = (raw >> 3) × 4mV                                  */
/* ========================================================================== */
HAL_StatusTypeDef INA219_ReadBusVoltage(float *voltage)
{
    uint16_t raw = 0;

    if (INA219_ReadReg(INA219_REG_BUS, &raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 检查 Bit 1 是否触发 Math Overflow (OVF) 算术溢出 
       如果置 1 说明计算超过上限，重新写一次 CALIB 防掉电清零 */
    if (raw & (1u << 1))
    {
        (void)INA219_WriteReg(INA219_REG_CALIB, INA219_CALIB_VALUE);
    }

    /* 计算实际电压: Bit[15:3] 为电压原始值 */
    float v_val = (float)(raw >> 3) * INA219_BUSV_LSB_V;

    /* 同步更新全局变量并返回 */
    ina219_vbus = v_val;
    if (voltage != NULL)
    {
        *voltage = v_val;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  读取母线电流 (A): 校准后电流寄存器为有符号 raw × Current_LSB               */
/* ========================================================================== */
HAL_StatusTypeDef INA219_ReadCurrent(float *current)
{
    uint16_t raw = 0;

    if (INA219_ReadReg(INA219_REG_CURRENT, &raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 转换 16 位有符号数并计算电流 */
    float i_val = (float)(int16_t)raw * INA219_CURRENT_LSB_A;

    /* 同步更新全局变量并返回 */
    ina219_ibus = i_val;
    if (current != NULL)
    {
        *current = i_val;
    }

    return HAL_OK;
}
