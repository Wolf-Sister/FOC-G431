/**
  ******************************************************************************
  * @file    as5047p_ext.h
  * @brief   AS5047P 编码器扩展 — 兼容 TinyFoc AS5600 API 模式的高级接口
  *          (角度、速度、多圈累积)
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

/* 速度估算器：80 个有效间隔 @ 20 kHz ≈ 4 ms */
#define AS5047P_VELOCITY_WINDOW_SIZE 80U


/* AS5047P 传感器句柄 -----------------------------------------------------*/
typedef struct {
    float    prev_angle;      /* 上一次机械角度 [0, 2pi)                    */
    int32_t  turn_count;      /* 整数圈计数器（无浮点漂移）                 */
    float    total_angle;     /* 多圈累积总角度 (rad)                       */
    float    velocity_rad_s;  /* 整数计数窗口速度 (rad/s)                   */
    uint16_t prev_raw;        /* 上一次 14 位编码器原始值                    */
    uint32_t prev_cycle;      /* 上一次有效更新的 DWT 周期计数               */
    uint32_t sample_cycle;    /* 当前角度帧实际开始采集时的 DWT 周期计数     */
    int16_t  velocity_count_window[AS5047P_VELOCITY_WINDOW_SIZE];
    uint32_t velocity_cycle_window[AS5047P_VELOCITY_WINDOW_SIZE];
    int32_t  velocity_count_sum;
    uint32_t velocity_cycle_sum;
    uint32_t total_errors;    /* 无效/丢失 DMA 采样总数                      */
    uint32_t consecutive_errors; /* 自上次有效帧以来连续无效采样数          */
    uint8_t  velocity_window_index;
    uint8_t  velocity_window_count;
    uint8_t  initialized;     /* 0 = 尚未收到有效角度帧，1 = 已初始化       */
} AS5047P_Sensor_T;

extern AS5047P_Sensor_T AngleSensor;

/* ========================================================================== */
/*  编码器缓存 — TIM2 ISR 写入 (优先级1)，FOC 读取 (优先级0)                  */
/* ========================================================================== */
typedef struct {
    float    angle_raw;         /* 单圈机械角度 [0, 2pi)                       */
    float    velocity_rad_s;    /* 机械角速度 [rad/s]                           */
    float    total_angle_rad;   /* 多圈累积总角度 [rad]                        */
    uint32_t sample_cycle;      /* 角度帧实际采集时刻的 DWT 周期计数           */
    uint32_t update_count;      /* 每次 TIM2 ISR 成功更新递增                   */
    uint8_t  data_valid;        /* 0 = 尚无有效数据, 1 = 缓存已就绪            */
} encoder_cache_t;


/* API ----------------------------------------------------------------------*/
void     AS5047P_Sensor_Init(AS5047P_Sensor_T *s);
bool     AS5047P_Sensor_Update(AS5047P_Sensor_T *s);
void     AS5047P_EncoderCache_Publish(const AS5047P_Sensor_T *s);
bool     AS5047P_EncoderCache_Read(encoder_cache_t *snapshot);
float    AS5047P_GetAngle(const AS5047P_Sensor_T *s);
float    AS5047P_GetVelocity(const AS5047P_Sensor_T *s);
float    AS5047P_GetAccumulateAngle(const AS5047P_Sensor_T *s);
float    AS5047P_GetOnceAngle(const AS5047P_Sensor_T *s);

#ifdef __cplusplus
}
#endif

#endif /* __AS5047P_EXT_H__ */
