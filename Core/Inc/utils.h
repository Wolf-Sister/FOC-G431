/**
  ******************************************************************************
  * @file    utils.h
  * @brief   FOC 工具函数 — DWT 定时器、滤波器、角度辅助
  ******************************************************************************
  */

#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 数学常量 ------------------------------------------------------------*/
#define PI           3.14159265358979f
#define _2PI         6.283185307f
#define _3PI_2       4.71238898038f
#define SQRT3        1.73205080756887729353f
#define _1_SQRT3     0.57735026919f
#define _2_SQRT3     1.15470053838f
#define SQRT3_BY_2   0.86602540378f
#define RAD_TO_DEG   57.295779513f   /* 180.0 / PI — ARM DSP sin_cos 接受角度制输入 */

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/* DWT 微秒定时器 -----------------------------------------------------*/
void           DWT_Init(void);
unsigned long  dwt_get_micros(void);
uint32_t       dwt_get_cycles(void);
float          dwt_cycles_to_seconds(uint32_t cycles);

/* 信号处理 ---------------------------------------------------------*/
float lowPassFilter(float new_val, float alpha, float *state);

/* 角度辅助函数 -----------------------------------------------------------*/
float _normalizeAngle(float angle);

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H__ */
