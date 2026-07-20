/**
  ******************************************************************************
  * @file    foc.h
  * @brief   FOC 电机控制 — SVPWM、电流环、传感器对齐、电机参数
  *
  *          将 TinyFoc 的 motor.h + foc.h 合并到现有 G431 foc 模块中
  ******************************************************************************
  */

#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "utils.h"
#include "as5047p_ext.h"
#include <stdbool.h>

/* 数学常量 — 已在 utils.h 中定义，此处保留以兼容旧代码 ---------*/
#ifndef PI
#define PI      3.1415926535f
#endif
#ifndef SQRT3
#define SQRT3   1.7320508075f
#endif

/* ========================================================================== */
/*  传感器 / 电流参数                                                          */
/* ========================================================================== */

/* 角度滤波器 --------------------------------------------------------------*/
#define FOC_LPF_ALPHA  0.15f

/* 电流采样 -----------------------------------------------------------*/
#define ADC_VOLTAGE_RANGE   3.3f
#define ADC_RESOLUTION      4096.0f
#define SHUNT_RESISTOR      0.01f
#define AMPLIFIER_GAIN      50.0f
#define CURRENT_FACTOR      (ADC_VOLTAGE_RANGE / (ADC_RESOLUTION * SHUNT_RESISTOR * AMPLIFIER_GAIN))

/* 电机供电 ---------------------------------------------------------------*/
#define MOTOR_VBUS                 12.0f  /* 电机供电电压 (V)                    */
#define ABSOLUTE_CURRENT_LIMIT     10.0f  /* 配置电流的硬上限                     */
#define CURRENT_VOLTAGE_LIMIT_DEFAULT 10.0f /* 电流环 dq 矢量限幅 (V)              */
#define SPEED_CURRENT_LIMIT_DEFAULT   0.60f /* 速度环 Iq 输出限幅 (A)              */

/* 传感器上电对齐 -----------------------------------------------------------*/
#define FOC_ALIGN_VOLTAGE_V          3.0f
#define FOC_ALIGN_HOLD_VOLTAGE_V     1.0f
#define FOC_ALIGN_RAMP_UP_MS         100U
#define FOC_ALIGN_HOLD_MS            300U
#define FOC_ALIGN_RAMP_DOWN_MS       50U
#define FOC_ALIGN_CURRENT_LIMIT_A    1.0f
#define FOC_ALIGN_OVERCURRENT_MS     3U

/* 速度环 -----------------------------------------------------------------*/
#define SPEED_DECIMATION   10         /* 20kHz / 10 = 2kHz 速度环             */
#define SPEED_LPF_ALPHA    0.0529f    /* 约 17 Hz 低通滤波器 alpha @ 2 kHz      */
#define SPEED_GAIN_SCHEDULE_DEFAULT       1U
#define SPEED_KP_LOW_SPEED_DEFAULT    0.090f
#define SPEED_KP_HIGH_SPEED_DEFAULT   0.020f
#define SPEED_KP_SWITCH_DOWN_RAD_S    8.0f
#define SPEED_KP_SWITCH_UP_RAD_S      9.0f
#define SPEED_GAIN_REGION_LOW_SPEED       0U
#define SPEED_GAIN_REGION_HIGH_SPEED      1U
#define FOC_ENCODER_PREDICTION_MAX_S  0.0002f /* 角度预测最大数据年龄 200 us */

/* 位置环 ---------------------------------------------------------------*/
#define POS_SPEED_LIMIT_DEFAULT       50.0f /* 位置环输出硬限幅 (rad/s)            */
#define POS_SPEED_LIMIT_MAX          600.0f /* 配置速度的硬上限                     */
#define POS_RELATIVE_STEP_MAX_RAD      6.2831853f
#define POS_ACCEL_LIMIT_DEFAULT       40.0f /* 位置轨迹加减速度 (rad/s^2)           */
#define POS_ACCEL_LIMIT_MAX         2000.0f
#define POSITION_LOOP_DT_S             0.001f

/* 电机电气参数 — 请根据实际电机调整! ------------------*/
#define MOTOR_Lq        0.0002f  /* q 轴电感 (H) — 典型 0.5 mH              */
#define MOTOR_Ld        0.0002f  /* d 轴电感 (H) — SPM 电机与 Lq 相同       */
#define MOTOR_FLUX      0.001815f   /* 永磁磁链 (Wb) — Ke=V·s/rad              */

/* Feed-forward is disabled at low speed and blended in at high speed. */
#define FOC_FEEDFORWARD_ENABLE    0U
#define FF_START_ELEC_RAD_S    1000.0f
#define FF_FULL_ELEC_RAD_S     1200.0f

/* ========================================================================== */
/*  相电流数据（双 ADC 采集，含校准状态）                                      */
/* ========================================================================== */
typedef struct {
    uint16_t Raw_A;
    uint16_t Raw_C;
    float    Offset_A;
    float    Offset_C;
    float    I_A;          /* A 相电流 (A)                                   */
    float    I_B;          /* B 相电流 (A) — 基尔霍夫推导                    */
    float    I_C;          /* C 相电流 (A)                                   */
    uint8_t  Calibrated;   /* 0 = 校准中, 1 = 就绪                           */
} Phase_Current_t;

extern volatile Phase_Current_t motor_current;

/* ========================================================================== */
/*  电机控制模式                                                               */
/* ========================================================================== */
typedef enum {
    MOTOR_TORQUE,          /* 转矩 / 电流闭环                                 */
    MOTOR_SPEED,           /* 速度闭环                                        */
    MOTOR_POSITION         /* 位置闭环                                        */
} Motor_Mode_e;

/* ========================================================================== */
/*  电机配置 & 控制状态 (来自 TinyFoc motor.h)                                 */
/* ========================================================================== */
typedef struct {
    float voltage_supply;       /* 最大母线电压                                 */
    int   dir;                  /* 电机方向 (+1 或 -1)                          */
    int   pairs;                /* 极对数                                       */
    float iq_p_gain;            /* Q 轴 (转矩) 电流环 P 增益                    */
    float iq_i_gain;            /* Q 轴 (转矩) 电流环 I 增益                    */
    float id_p_gain;            /* D 轴 (磁通) 电流环 P 增益                    */
    float id_i_gain;            /* D 轴 (磁通) 电流环 I 增益                    */
    float spd_p_gain;           /* 速度环 P 增益                                */
    float spd_i_gain;           /* 速度环 I 增益                                */
    float spd_p_low_speed;      /* 调度低速区 P 增益                            */
    float spd_p_high_speed;     /* 调度高速区 P 增益                            */
    uint8_t spd_gain_schedule;  /* 1 = 启用两段速度 P 增益调度                  */
    float pos_p_gain;           /* 位置环 P 增益 (rad/s per rad)                */
    float current_voltage_limit;/* 电流环 dq 电压矢量限幅 (V)                   */
    float speed_current_limit;  /* 速度环 Iq 输出/电流限幅 (A)                  */
    float pos_speed_limit;      /* 位置环速度输出限幅 (rad/s)                   */
    float pos_accel_limit;      /* 位置轨迹加减速度限幅 (rad/s^2)               */
} motor_config_t;

typedef struct {
    /* 相电流 (A) — 在 ADC ISR 中从 motor_current 填充                       */
    float IphA;
    float IphB;
    float IphC;
    uint32_t IphA_offset;
    uint32_t IphB_offset;
    uint32_t IphC_offset;

    /* 目标设定值 */
    float set_torque;

    /* 模式 & 校准 */
    uint8_t mode;
    float   zero_elec_angle;
    bool    pre_calibrated;
    bool    encoder_updated;

    /* dq 轴 */
    float iq_set;             /* PID 输出 → Uq 指令 (V)                       */
    float id_set;             /* PID 输出 → Ud 指令 (V)                       */
    float iq_meas;            /* 测量 Iq (滤波后)                              */
    float id_meas;            /* 测量 Id (滤波后)                              */
    float id_target;          /* D 轴电流目标 (A)，SPM 电机默认为 0            */
    float set_speed;           /* 速度目标值 (机械角速度 rad/s)                 */
    float set_position;        /* 位置目标值 (多圈弧度 rad)                     */
    float pos_meas;            /* 测量多圈位置 (rad)                            */
    float vel_meas;            /* 测量速度，滤波后 (rad/s)                      */
    float vel_raw;             /* 滤波前原始速度 (rad/s)                        */
    float vel_filter_state;    /* 速度低通滤波状态变量                          */
    uint8_t spd_needs_init;    /* 1 = 在控制 ISR 中重置速度滤波器/PID          */
    float spd_kp_active;        /* 当前速度环实际使用的 P 增益                   */
    uint8_t spd_gain_region;    /* SPEED_GAIN_REGION_*                          */
    float mod_q;              /* 归一化 q 轴调制                              */
    float mod_d;              /* 归一化 d 轴调制                              */

    /* 步进同步标志：命令解析器置位，遥测发送后清除                             */
    uint8_t status_flag;

    /* 电流滤波器状态（实例化，避免静态冲突）                                  */
    float iq_filter_state;
    float id_filter_state;

    /* PWM 占空比 (0~1) */
    float du, dv, dw;

    /* 原始 ADC 快照 (调试用) */
    uint32_t latest_ib_raw;
    uint32_t latest_ic_raw;
} motor_control_t;

extern motor_config_t  motor_config;
extern motor_control_t motor_control;

/* 电流环使能标志 — 校准 + 对齐完成后由 main 设置                              */
extern volatile uint8_t current_loop_enable;

/* Commands are accepted after main completes all control initialization. */
extern volatile uint8_t motor_ready;

/* 传感器对齐进行中 — TIM 回调不得覆盖 PWM                                     */
extern volatile uint8_t alignment_in_progress;

/* CORDIC sin/cos 缓存 — 当前帧轮询成功时更新，超时时作为安全回退                */
extern volatile float cordic_sin_cache;
extern volatile float cordic_cos_cache;
#define CORDIC_Q31_PER_RAD  683565275.0f   /* 2^31 / PI                         */
#define FOC_CORDIC_POLL_LIMIT  32U         /* 当前帧结果最大短轮询次数             */

/* ========================================================================== */
/*  函数原型                                                                   */
/* ========================================================================== */

/* --- 现有（保留） -------------------------------------------------------*/
void  SVPWM_Update(float Ud, float Uq, float angle, uint32_t period);
void  Motor_Current_Calibration(void);
void  UART2_SendString(const char *str);

/* --- 电机控制 ---------------------------------------------------------*/
void  motor_control_parm_init(void);
void  foc_set_loop_limits(float current_voltage_limit,
                          float speed_current_limit,
                          float position_speed_limit);

/* --- Clarke + Park 变换 -----------------------------------------------*/
float cal_Iq_Id(float cur_b, float cur_c, float angle_el);

/* --- 传感器对齐 ------------------------------------------------------*/
HAL_StatusTypeDef foc_alignSensor(void);

/* --- 闭环控制 ---------------------------------------------------*/
void  foc_current_loop(void);

/* --- 位置外环 (1 kHz, TIM3 ISR) ---------------------------------*/
void  foc_position_loop(void);

/* --- SVPWM 前向通道 (d,q → PWM) ----------------------------------------*/
void  foc_forward(float d, float q, float angle_el);

/* --- 电气角度辅助函数 ----------------------------------------------*/
float _electricalAngle(void);
float _electricalVelocity(void);
float _calculate_zero_electric_angle(void);

/* --- ADC → 相电流同步 ----------------------------------------------*/
void  foc_sync_phase_currents(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
