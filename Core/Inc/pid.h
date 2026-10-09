/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID 控制器 — Tustin 积分器、输出速率限制、抗积分饱和
  ******************************************************************************
  */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Default loop tuning -------------------------------------------------------*/
#define IQ_CURRENT_KP_DEFAULT     10.4850f
#define IQ_CURRENT_KI_DEFAULT   371.2500f
#define ID_CURRENT_KP_DEFAULT     10.4850f
#define ID_CURRENT_KI_DEFAULT   371.2500f
#define SPEED_KP_DEFAULT          0.0900f
#define SPEED_KI_DEFAULT          0.1000f
#define POSITION_KP_DEFAULT      10.0000f
#define CURRENT_VECTOR_AW_GAIN_DEFAULT  2000.0f

/* PID 控制器状态 ------------------------------------------------------*/
struct PIDController {
    float P;                  /* 比例增益                                  */
    float I;                  /* 积分增益                                  */
    float D;                  /* 微分增益                                  */
    float output_ramp;        /* 最大输出速率 (units/sec)，0=关闭           */
    float limit;              /* 输出饱和限制 (+/-)                         */

    float error_prev;         /* 上一次误差（用于微分和 Tustin 积分）       */
    float output_prev;        /* 上一次输出（用于斜坡限制）                 */
    float integral_prev;      /* 上一次积分器状态                           */
    uint32_t timestamp_prev_cycles; /* 上一次更新时间戳 (DWT 周期)           */
    float sample_time;         /* Last validated controller sample time (s). */
};

/* 全局 PID 实例 -----------------------------------------------------*/
extern struct PIDController current_loop;     /* Iq 电流环                   */
extern struct PIDController id_current_loop;  /* Id 电流环                   */
extern struct PIDController speed_loop;       /* 速度环                      */

/* API ----------------------------------------------------------------------*/
float PIDController_Update(struct PIDController *pid, float error);
void  PIDController_ApplyTracking(struct PIDController *pid,
                                  float output_correction, float tracking_gain);

void  control_pid_init(void);
void  motor_pid_init(float iq_p, float iq_i, float id_p, float id_i);
void  speed_pid_init(float spd_p, float spd_i);
void  speed_pid_reset(void);
void  foc_set_current_pid(float P, float I, float D, float ramp);
void  foc_set_id_current_pid(float P, float I, float D, float ramp);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
