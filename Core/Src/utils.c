/**
  ******************************************************************************
  * @file    utils.c
  * @brief   FOC 工具函数实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include <math.h>
#include "arm_math.h"

/* 私有变量 ---------------------------------------------------------*/
static uint32_t cpu_freq_mhz = 0;

/* --------------------------------------------------------------------------*/
/**
  * @brief  初始化 DWT 周期计数器，用于微秒级计时
  */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT      = 0;
    cpu_freq_mhz     = SystemCoreClock / 1000000;
}

/**
  * @brief  读取原始 32 位 DWT 周期计数值
  */
uint32_t dwt_get_cycles(void)
{
    return DWT->CYCCNT;
}

/**
  * @brief  将 DWT 周期差值（无符号）转换为秒
  */
float dwt_cycles_to_seconds(uint32_t cycles)
{
    return (float)cycles / (float)SystemCoreClock;
}

/**
  * @brief  获取自 DWT_Init 以来的短期微秒级时间
  * @note   跨溢出场景下建议直接使用原始周期差值
  */
unsigned long dwt_get_micros(void)
{
    return dwt_get_cycles() / cpu_freq_mhz;
}

/**
  * @brief  一阶低通滤波器（实例化接口）
  * @param  new_val  新采样值
  * @param  alpha    平滑系数（0～1，越小滤波越强）
  * @param  state    指向持久化滤波状态的指针
  * @retval 滤波后的值
  */
float lowPassFilter(float new_val, float alpha, float *state)
{
    *state = alpha * new_val + (1.0f - alpha) * (*state);
    return *state;
}

/**
  * @brief  将角度归一化到 [0, 2*PI) 范围
  */
float _normalizeAngle(float angle)
{
    float a = fmodf(angle, _2PI);
    return (a < 0) ? a + _2PI : a;
}
