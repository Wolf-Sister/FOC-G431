/**
  ******************************************************************************
  * @file    foc.h
  * @brief   FOC motor control — SVPWM, current loop, sensor alignment, motor params
  *
  *          Merges TinyFoc's motor.h + foc.h into the existing G431 foc module.
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

/* Math constants — now defined in utils.h, kept for backward compat ---------*/
#ifndef PI
#define PI      3.1415926535f
#endif
#ifndef SQRT3
#define SQRT3   1.7320508075f
#endif

/* ========================================================================== */
/*  Sensor / current parameters                                               */
/* ========================================================================== */

/* Angle filter --------------------------------------------------------------*/
#define FOC_LPF_ALPHA  0.15f

/* Current sensing -----------------------------------------------------------*/
#define ADC_VOLTAGE_RANGE   3.3f
#define ADC_RESOLUTION      4096.0f
#define SHUNT_RESISTOR      0.01f
#define AMPLIFIER_GAIN      50.0f
#define CURRENT_FACTOR      (ADC_VOLTAGE_RANGE / (ADC_RESOLUTION * SHUNT_RESISTOR * AMPLIFIER_GAIN))

/* Motor power ---------------------------------------------------------------*/
#define MOTOR_VBUS      12       /* Motor supply voltage (V)                  */
#define LIMIT_CURRENT   5.0f     /* Max phase current (A)                     */
#define VEL_ALPHA       0.05f    /* Velocity low-pass filter coefficient      */
#define KV_FF           0.002f   /* Velocity feed-forward gain                */
#define VEL_DEADBAND    1.0f     /* Velocity dead-zone threshold              */

/* ========================================================================== */
/*  Open-loop control (kept from existing G431 code)                          */
/* ========================================================================== */
typedef struct {
    float Uq;
    float Ud;
    float Angle;
    float Speed;
    uint32_t Period;
} OpenLoop_Ctrl_t;

extern OpenLoop_Ctrl_t motor_ctrl;

/* ========================================================================== */
/*  Phase current data (dual-ADC captured, calibration state)                 */
/* ========================================================================== */
typedef struct {
    uint16_t Raw_A;
    uint16_t Raw_C;
    float    Offset_A;
    float    Offset_C;
    float    I_A;          /* Phase A current (A)                             */
    float    I_B;          /* Phase B current (A) — Kirchhoff derived         */
    float    I_C;          /* Phase C current (A)                             */
    uint8_t  Calibrated;   /* 0 = calibrating, 1 = ready                     */
} Phase_Current_t;

extern volatile Phase_Current_t motor_current;

/* ========================================================================== */
/*  Motor mode                                                                */
/* ========================================================================== */
typedef enum {
    MOTOR_POSITION,        /* Position closed loop                            */
    MOTOR_SPEED,           /* Speed closed loop                               */
    MOTOR_TORQUE,          /* Torque / current closed loop                    */
} Motor_Mode_e;

/* ========================================================================== */
/*  Motor configuration & control state (from TinyFoc motor.h)                 */
/* ========================================================================== */
typedef struct {
    float voltage_supply;       /* Max bus voltage                             */
    int   dir;                  /* Motor direction (+1 or -1)                   */
    int   pairs;                /* Pole pairs                                   */
    float pos_gain;             /* Position-loop P gain                        */
    float vel_gain;             /* Velocity-loop P gain                        */
    float vel_integrator_gain;  /* Velocity-loop I gain                        */
    float torque_gain;          /* Current-loop P gain                         */
    float torque_integrator_gain; /* Current-loop I gain                       */
} motor_config_t;

typedef struct {
    /* Phase currents (A) — populated from motor_current in ADC ISR           */
    float IphA;
    float IphB;
    float IphC;
    uint32_t IphA_offset;
    uint32_t IphB_offset;
    uint32_t IphC_offset;

    /* Setpoints */
    float set_pos;
    float set_vel;
    float set_torque;
    float Iq_target;
    float pos_target;
    float vel_target;

    /* Mode & calibration */
    uint8_t mode;
    float   zero_elec_angle;
    bool    pre_calibrated;
    bool    encoder_updated;
    int32_t pos_abs;

    /* dq-axis */
    float iq_set;             /* PID output → Uq command                       */
    float iq_meas;            /* Measured Iq (filtered)                        */
    float id_meas;            /* Measured Id (filtered)                        */
    float mod_q;              /* Normalized q-axis modulation                  */

    /* PWM duty (0~1) */
    float du, dv, dw;

    /* Velocity filter alpha */
    float vel_lowpass_alpha;

    /* Raw ADC snapshots (debug) */
    uint32_t latest_ib_raw;
    uint32_t latest_ic_raw;
} motor_control_t;

extern motor_config_t  motor_config;
extern motor_control_t motor_control;

/* Current loop enable flag — set by main after calibration + alignment       */
extern volatile uint8_t current_loop_enable;

/* Sensor alignment in progress — TIM callback must not overwrite PWM          */
extern volatile uint8_t alignment_in_progress;

/* ========================================================================== */
/*  Function prototypes                                                       */
/* ========================================================================== */

/* --- Existing (kept) -------------------------------------------------------*/
void  SVPWM_Update(float Ud, float Uq, float angle, uint32_t period);
void  Motor_Current_Calibration(void);
float FOC_GetSmoothAngle(void);
void  UART2_SendString(const char *str);

/* --- Motor control ---------------------------------------------------------*/
void  motor_control_parm_init(void);
void  set_motor_mode(Motor_Mode_e mode);

/* --- Clarke + Park transform -----------------------------------------------*/
float cal_Iq_Id(float cur_b, float cur_c, float angle_el);

/* --- Sensor alignment ------------------------------------------------------*/
void  foc_alignSensor(float q_voltage);

/* --- Closed-loop control ---------------------------------------------------*/
void  foc_current_loop(void);
void  foc_velocity_loop(void);
void  foc_position_loop(void);

/* --- SVPWM forward path (d,q → PWM) ----------------------------------------*/
void  foc_forward(float d, float q, float angle_el);

/* --- Electrical angle helpers ----------------------------------------------*/
float _electricalAngle(void);
float _electricalVelocity(void);
float _calculate_zero_electric_angle(void);

/* --- ADC → phase current sync ----------------------------------------------*/
void  foc_sync_phase_currents(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
