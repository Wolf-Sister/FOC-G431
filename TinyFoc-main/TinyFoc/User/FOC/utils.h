#ifndef __UTILS__H
#define __UTILS__H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "motor.h"

#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define rad60 deg2rad(60)

#define SQRT3 1.73205080756887729353
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define SQRT3_BY_2  0.86602540378f

#define _3PI_2 4.71238898038f
#define _2PI (6.283185306f)

#define min(x, y) (((x) > (y)) ? (x) : (y))
#define max(x, y) (((x) < (y)) ? (x) : (y))

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/**
 * @brief 初始化 DWT 外设
 */
void DWT_Init(void);

/**
 * @brief 获取微秒时间戳 (20s左右溢出一次)
 * @return unsigned long 当前微秒数
 */
unsigned long dwt_get_micros(void);

/**
 * @brief 一阶低通滤波器
 * @return float 经过平滑后的值
 */
float lowPassFilter(float new_angle, float alpha) ;

float _normalizeAngle(float angle);

float _electricalAngle(void);
float _calculate_zero_electric_angle(void);

float _electricalVelocity(void);

#endif

