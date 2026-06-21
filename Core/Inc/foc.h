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
#define LIMIT_CURRENT   10.0f    /* Max phase current (A)                     */

/* Motor electrical parameters — TUNE THESE for your motor! ------------------*/
#define MOTOR_Lq        0.0005f  /* q-axis inductance (H) — 0.5 mH typical    */
#define MOTOR_Ld        0.0005f  /* d-axis inductance (H) — same for SPM      */
#define MOTOR_FLUX      0.005f   /* PM flux linkage (Wb) — Ke=V·s/rad         */

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
    MOTOR_TORQUE,          /* Torque / current closed loop                    */
} Motor_Mode_e;

/* ========================================================================== */
/*  Motor configuration & control state (from TinyFoc motor.h)                 */
/* ========================================================================== */
typedef struct {
    float voltage_supply;       /* Max bus voltage                             */
    int   dir;                  /* Motor direction (+1 or -1)                   */
    int   pairs;                /* Pole pairs                                   */
    float iq_p_gain;            /* Q-axis (torque) current-loop P gain         */
    float iq_i_gain;            /* Q-axis (torque) current-loop I gain         */
    float id_p_gain;            /* D-axis (flux)   current-loop P gain         */
    float id_i_gain;            /* D-axis (flux)   current-loop I gain         */
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
    float set_torque;

    /* Mode & calibration */
    uint8_t mode;
    float   zero_elec_angle;
    bool    pre_calibrated;
    bool    encoder_updated;

    /* dq-axis */
    float iq_set;             /* PID output → Uq command (V)                   */
    float id_set;             /* PID output → Ud command (V)                   */
    float iq_meas;            /* Measured Iq (filtered)                        */
    float id_meas;            /* Measured Id (filtered)                        */
    float id_target;          /* D-axis current target (A), default 0 for SPM   */
    float mod_q;              /* Normalized q-axis modulation                  */
    float mod_d;              /* Normalized d-axis modulation                  */

    /* Step-sync flag: set by command parser, cleared after telemetry TX        */
    uint8_t status_flag;

    /* Current filter state (instance-based, avoids static clash)             */
    float iq_filter_state;
    float id_filter_state;

    /* PWM duty (0~1) */
    float du, dv, dw;

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

/* CORDIC sin/cos cache — CORDIC ISR writes, TIM1 ISR reads                     */
extern volatile float cordic_sin_cache;
extern volatile float cordic_cos_cache;
#define CORDIC_Q31_PER_RAD  683565275.0f   /* 2^31 / PI                         */

/* ========================================================================== */
/*  Function prototypes                                                       */
/* ========================================================================== */

/* --- Existing (kept) -------------------------------------------------------*/
void  SVPWM_Update(float Ud, float Uq, float angle, uint32_t period);
void  Motor_Current_Calibration(void);
void  UART2_SendString(const char *str);

/* --- Motor control ---------------------------------------------------------*/
void  motor_control_parm_init(void);

/* --- Clarke + Park transform -----------------------------------------------*/
float cal_Iq_Id(float cur_b, float cur_c, float angle_el);

/* --- Sensor alignment ------------------------------------------------------*/
void  foc_alignSensor(float q_voltage);

/* --- Closed-loop control ---------------------------------------------------*/
void  foc_current_loop(void);

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
