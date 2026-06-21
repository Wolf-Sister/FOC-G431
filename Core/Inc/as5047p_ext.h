/**
  ******************************************************************************
  * @file    as5047p_ext.h
  * @brief   AS5047P encoder extension — high-level interface compatible with
  *          TinyFoc's AS5600 API pattern (angle, velocity, multi-turn accumulation)
  ******************************************************************************
  */

#ifndef __AS5047P_EXT_H__
#define __AS5047P_EXT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* AS5047P sensor handle -----------------------------------------------------*/
typedef struct {
    float    prev_angle;      /* Previous raw angle (0–2π)                   */
    int32_t  turn_count;      /* Integer full-turn counter (no float drift)  */
    float    total_angle;     /* Multi-turn total angle (rad)                */
    float    velocity_rad_s;  /* Angular velocity (rad/s)                    */
    uint32_t prev_ts;         /* Timestamp of last update (us)               */
} AS5047P_Sensor_T;

extern AS5047P_Sensor_T AngleSensor;

/* ========================================================================== */
/*  Encoder cache — written by TIM2 ISR (priority 1), read by FOC (priority 0) */
/* ========================================================================== */
typedef struct {
    volatile float    angle_raw;         /* Single-turn mechanical angle [0, 2pi)       */
    volatile float    velocity_rad_s;    /* Mechanical angular velocity [rad/s]          */
    volatile float    total_angle_rad;   /* Multi-turn accumulated total angle [rad]     */
    volatile uint32_t update_count;      /* Incremented each successful TIM2 ISR update  */
    volatile uint8_t  data_valid;        /* 0 = no valid data yet, 1 = cache is live     */
} encoder_cache_t;

extern volatile encoder_cache_t encoder_cache;

/* API ----------------------------------------------------------------------*/
void     AS5047P_Sensor_Init(AS5047P_Sensor_T *s);
void     AS5047P_Sensor_Update(AS5047P_Sensor_T *s);
float    AS5047P_GetAngle(const AS5047P_Sensor_T *s);
float    AS5047P_GetVelocity(const AS5047P_Sensor_T *s);
float    AS5047P_GetAccumulateAngle(const AS5047P_Sensor_T *s);
float    AS5047P_GetOnceAngle(const AS5047P_Sensor_T *s);

#ifdef __cplusplus
}
#endif

#endif /* __AS5047P_EXT_H__ */
