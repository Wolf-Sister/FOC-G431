/**
  ******************************************************************************
  * @file    as5047p_ext.c
  * @brief   AS5047P encoder extension — velocity, multi-turn accumulation
  *
  *          Usage pattern (identical to TinyFoc's AS5600):
  *            1. AS5047P_Sensor_Init(&AngleSensor);
  *            2. Call AS5047P_Sensor_Update(&AngleSensor) at fixed rate (1kHz)
  *            3. Read with AS5047P_GetAngle / GetVelocity / GetAccumulateAngle
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "as5047p_ext.h"
#include "as5047p.h"
#include "utils.h"

/* Conversion constant: AS5047P is 14-bit (0–16383) → 0–2π radians -----------*/
#define AS5047P_ANGLE_MAX 16383.0f
#define AS5047P_RAD_SCALE (_2PI / AS5047P_ANGLE_MAX)

/* Global sensor instance ----------------------------------------------------*/
AS5047P_Sensor_T AngleSensor = {0};

/* Encoder cache — populated by TIM2 ISR @10kHz, consumed by FOC @20kHz */
volatile encoder_cache_t encoder_cache = {0};

/* --------------------------------------------------------------------------*/
/**
  * @brief  Initialize sensor state
  */
void AS5047P_Sensor_Init(AS5047P_Sensor_T *s)
{
    s->prev_angle     = 0.0f;
    s->turn_count     = 0;
    s->total_angle    = 0.0f;
    s->velocity_rad_s = 0.0f;
    s->prev_ts        = 0;
}

/**
  * @brief  Update sensor reading — call at fixed rate (e.g. 1kHz TIM ISR)
  *
  *         1. Reads latest DMA-captured raw angle
  *         2. Detects wrap-around for multi-turn accumulation
  *         3. Computes velocity from delta-angle / delta-time
  */
void AS5047P_Sensor_Update(AS5047P_Sensor_T *s)
{
    uint16_t raw = AS5047P_DMA_GetAngleCallback();
    if (raw == 0xFFFFU) return;

    float angle = (float)raw * AS5047P_RAD_SCALE;
    uint32_t now = dwt_get_micros();

    /* first init */
    if (s->prev_ts == 0) {
        s->prev_angle  = angle;
        s->total_angle = angle;
        s->turn_count  = 0;
        s->prev_ts     = now;
        return;
    }

    float delta = angle - s->prev_angle;

    /* ================================
       INTEGER WRAP DETECTION CORE
       ================================ */
    if (delta > PI) {
        s->turn_count--;            /* reverse cross: 0 → 2π */
        delta -= _2PI;
    }
    else if (delta < -PI) {
        s->turn_count++;            /* forward cross: 2π → 0  */
        delta += _2PI;
    }

    /* total angle = integer turns + fractional */
    s->total_angle = (float)s->turn_count * _2PI + angle;

    /* velocity (delta already corrected for wrap) */
    float dt = (float)(now - s->prev_ts) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) {
        s->velocity_rad_s = delta / dt;
    }

    s->prev_angle = angle;
    s->prev_ts    = now;

    AS5047P_DMA_StartRequest();
}

/**
  * @brief  Get current mechanical angle [0, 2π)
  */
float AS5047P_GetAngle(const AS5047P_Sensor_T *s)
{
    return s->prev_angle;
}

/**
  * @brief  Get instantaneous angular velocity [rad/s]
  */
float AS5047P_GetVelocity(const AS5047P_Sensor_T *s)
{
    return s->velocity_rad_s;
}

/**
  * @brief  Get accumulated multi-turn angle [rad]
  */
float AS5047P_GetAccumulateAngle(const AS5047P_Sensor_T *s)
{
    return s->total_angle;
}

/**
  * @brief  Get raw angle, single conversion (blocking, for calibration use only)
  */
float AS5047P_GetOnceAngle(const AS5047P_Sensor_T *s)
{
    uint16_t raw = AS5047P_DMA_GetAngleCallback();
    if (raw == 0xFFFFU) return s->prev_angle;
    return (float)raw * AS5047P_RAD_SCALE;
}
