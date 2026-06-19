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
    /* --- State --- */
    float        prev_angle_raw;      /* Previous raw angle (0–2π)             */
    unsigned long prev_update_ts;     /* Timestamp of last update (us)          */

    /* --- Outputs --- */
    float        rotation_offset;     /* Accumulated full-rotation offset (rad) */
    float        total_angle_rad;     /* Multi-turn total angle (rad)           */
    float        velocity_rad_s;      /* Angular velocity (rad/s)               */
} AS5047P_Sensor_T;

extern AS5047P_Sensor_T AngleSensor;

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
