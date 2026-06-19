/**
  ******************************************************************************
  * @file    utils.h
  * @brief   FOC utility functions — DWT timer, filters, angle helpers
  ******************************************************************************
  */

#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Math constants ------------------------------------------------------------*/
#define PI           3.14159265358979f
#define _2PI         6.283185307f
#define _3PI_2       4.71238898038f
#define SQRT3        1.73205080756887729353f
#define _1_SQRT3     0.57735026919f
#define _2_SQRT3     1.15470053838f
#define SQRT3_BY_2   0.86602540378f

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/* DWT microsecond timer -----------------------------------------------------*/
void           DWT_Init(void);
unsigned long  dwt_get_micros(void);

/* Signal processing ---------------------------------------------------------*/
float lowPassFilter(float new_val, float alpha, float *state);

/* Angle utilities -----------------------------------------------------------*/
float _normalizeAngle(float angle);

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H__ */
