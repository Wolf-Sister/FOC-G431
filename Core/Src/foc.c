/**
  ******************************************************************************
  * @file    foc.c
  * @brief   FOC motor control — SVPWM, current loop, sensor alignment
  *
  *          Merges TinyFoc's motor.c + foc.c into the existing G431 module.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "foc.h"
#include "tim.h"
#include "usart.h"
#include "as5047p.h"
#include "pid.h"
#include <math.h>
#include "arm_math.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*  Global variables                                                          */
/* ========================================================================== */

/* Phase currents (dual-ADC captured) */
volatile Phase_Current_t motor_current = {0};

/* Current loop enable — set after calibration + alignment done */
volatile uint8_t current_loop_enable = 0;

/* Sensor alignment in progress — TIM callback must not overwrite PWM */
volatile uint8_t alignment_in_progress = 0;

/* Motor config (from TinyFoc) */
motor_config_t motor_config = {
    .voltage_supply         = MOTOR_VBUS,
    .dir                    = -1,
    .pairs                  = 11,
    .iq_p_gain              = 0,
    .iq_i_gain              = 0,
    .id_p_gain              = 0,
    .id_i_gain              = 0,
    .spd_p_gain             = 0.05f,
    .spd_i_gain             = 2.0f,
};

/* Motor control state (from TinyFoc) */
motor_control_t motor_control = {
    .IphA              = 0.0f,
    .IphB              = 0.0f,
    .IphC              = 0.0f,
    .IphA_offset       = 0,
    .IphB_offset       = 0,
    .IphC_offset       = 0,
    .set_torque        = 0.0f,
    .mode              = MOTOR_TORQUE,
    .zero_elec_angle   = 0.0f,
    .pre_calibrated    = false,
    .encoder_updated   = false,
    .iq_set            = 0.0f,
    .id_set            = 0.0f,
    .iq_meas           = 0.0f,
    .id_meas           = 0.0f,
    .id_target         = 0.0f,
    .set_speed         = 0.0f,
    .vel_meas          = 0.0f,
    .vel_raw           = 0.0f,
    .vel_filter_state  = 0.0f,
    .mod_q             = 0.0f,
    .mod_d             = 0.0f,
    .du                = 0.0f,
    .dv                = 0.0f,
    .dw                = 0.0f,
    .latest_ib_raw     = 0,
    .latest_ic_raw     = 0,
};

/* CORDIC sin/cos cache — written by CORDIC ISR, read by foc_current_loop()    */
volatile float cordic_sin_cache = 0.0f;
volatile float cordic_cos_cache = 1.0f;  /* cos(0)=1 safe initial value         */

/* ========================================================================== */
/*  Existing SVPWM (kept for backward compat / open-loop testing)             */
/* ========================================================================== */

/**
  * @brief  11-segment SVPWM update — writes CCR1/2/3 for TIM1
  */
void SVPWM_Update(float Ud, float Uq, float angle, uint32_t period)
{
    float sin_angle, cos_angle;
    arm_sin_cos_f32(angle * RAD_TO_DEG, &sin_angle, &cos_angle);
    float Ualpha = Ud * cos_angle - Uq * sin_angle;
    float Ubeta  = Ud * sin_angle + Uq * cos_angle;

    uint8_t sector = 0;
    float v1 = Ubeta;
    float v2 = (SQRT3 * Ualpha - Ubeta) / 2.0f;
    float v3 = (-SQRT3 * Ualpha - Ubeta) / 2.0f;

    if (v1 > 0) sector += 1;
    if (v2 > 0) sector += 2;
    if (v3 > 0) sector += 4;

    switch (sector) {
        case 3: sector = 1; break;
        case 1: sector = 2; break;
        case 5: sector = 3; break;
        case 4: sector = 4; break;
        case 6: sector = 5; break;
        case 2: sector = 6; break;
        default: return;
    }

    float Tlow = (float)period;
    float X = SQRT3 * Tlow * Ubeta;
    float Y = (3.0f * Ualpha + SQRT3 * Ubeta) * Tlow / 2.0f;
    float Z = (-3.0f * Ualpha + SQRT3 * Ubeta) * Tlow / 2.0f;

    float t1 = 0.0f, t2 = 0.0f;
    switch (sector) {
        case 1: t1 = -Z; t2 =  X; break;
        case 2: t1 =  Y; t2 =  Z; break;
        case 3: t1 =  X; t2 = -Y; break;
        case 4: t1 =  Z; t2 = -X; break;
        case 5: t1 = -Y; t2 = -Z; break;
        case 6: t1 = -X; t2 =  Y; break;
    }

    float sum = t1 + t2;
    if (sum > Tlow) {
        t1 = t1 * Tlow / sum;
        t2 = t2 * Tlow / sum;
    }

    float ta = (Tlow - t1 - t2) / 4.0f;
    float tb = ta + t1 / 2.0f;
    float tc = tb + t2 / 2.0f;

    uint16_t ccr1 = 0, ccr2 = 0, ccr3 = 0;
    switch (sector) {
        case 1: ccr1 = (uint16_t)ta; ccr2 = (uint16_t)tb; ccr3 = (uint16_t)tc; break;
        case 2: ccr1 = (uint16_t)tb; ccr2 = (uint16_t)ta; ccr3 = (uint16_t)tc; break;
        case 3: ccr1 = (uint16_t)tc; ccr2 = (uint16_t)ta; ccr3 = (uint16_t)tb; break;
        case 4: ccr1 = (uint16_t)tc; ccr2 = (uint16_t)tb; ccr3 = (uint16_t)ta; break;
        case 5: ccr1 = (uint16_t)tb; ccr2 = (uint16_t)tc; ccr3 = (uint16_t)ta; break;
        case 6: ccr1 = (uint16_t)ta; ccr2 = (uint16_t)tc; ccr3 = (uint16_t)tb; break;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}

/* ========================================================================== */
/*  Calibration & angle filter (kept)                                         */
/* ========================================================================== */

void Motor_Current_Calibration(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    UART2_SendString("[FOC] Calibrating Current Sensor Offset via Interrupt...\r\n");

    motor_current.Offset_A  = 0;
    motor_current.Offset_C  = 0;
    motor_current.Calibrated = 0;
}

void UART2_SendString(const char *str)
{
    /* Wait for previous DMA transfer to complete (DMA_NORMAL mode:
     * TC ISR restores gState to READY) */
    while (huart2.gState != HAL_UART_STATE_READY) {}
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)str, strlen(str));
}

/* ========================================================================== */
/*  Phase current sync — copy ADC-captured currents to motor_control          */
/* ========================================================================== */

/**
  * @brief  Sync phase currents from dual-ADC buffers to motor_control struct.
  *         Called in ADC injection callback before current loop.
  */
void foc_sync_phase_currents(void)
{
    motor_control.IphA = motor_current.I_A;
    motor_control.IphB = motor_current.I_B;
    motor_control.IphC = motor_current.I_C;
}

/* ========================================================================== */
/*  Motor parameter init                                                      */
/* ========================================================================== */

void motor_control_parm_init(void)
{
    motor_control.iq_set     = 0.0f;
    motor_control.id_set     = 0.0f;
    motor_control.iq_meas    = 0.0f;
    motor_control.id_meas    = 0.0f;
    motor_control.id_target  = 0.0f;
    motor_control.status_flag = 0;
    motor_control.id_filter_state = 0.0f;
    motor_control.iq_filter_state = 0.0f;
    motor_control.set_speed        = 0.0f;
    motor_control.vel_meas         = 0.0f;
    motor_control.vel_raw          = 0.0f;
    motor_control.vel_filter_state = 0.0f;
    motor_control.spd_prev_angle   = encoder_cache.total_angle_rad;
    motor_control.spd_needs_init   = 1;
}

/* ========================================================================== */
/*  Electrical angle helpers (from TinyFoc utils.c)                            */
/* ========================================================================== */

/**
  * @brief  Calculate zero-electric-angle offset during alignment
  */
float _calculate_zero_electric_angle(void)
{
    float sum_angle = 0.0f;
    for (int i = 0; i < 10; i++) {
        AS5047P_Sensor_Update(&AngleSensor);
        sum_angle += AS5047P_GetAngle(&AngleSensor);
        HAL_Delay(1);
    }
    float mech_angle = sum_angle / 10.0f;

    float raw_elec_angle = (float)(motor_config.dir * motor_config.pairs) * mech_angle;
    return _normalizeAngle(raw_elec_angle);
}

/**
  * @brief  Get instantaneous electrical angle [0, 2π)
  */
float _electricalAngle(void)
{
    float mech_angle = encoder_cache.angle_raw;  /* volatile read from TIM2 ISR cache */
    float elec_angle = (float)(motor_config.dir * motor_config.pairs)
                       * mech_angle
                       - motor_control.zero_elec_angle;
    return _normalizeAngle(elec_angle);
}

/**
  * @brief  Get electrical angular velocity [rad/s]
  */
float _electricalVelocity(void)
{
    float mech_vel = encoder_cache.velocity_rad_s;  /* volatile read from TIM2 ISR cache */
    return (float)(motor_config.dir * motor_config.pairs) * mech_vel;
}

/* ========================================================================== */
/*  Clarke + Park transform (from TinyFoc motor.c)                             */
/* ========================================================================== */

/**
  * @brief  Compute Iq from phase B & C currents via Clarke + Park
  * @param  cur_b    B-phase current (A)
  * @param  cur_c    C-phase current (A)
  * @param  angle_el electrical angle [rad]
  * @return Iq (torque-producing current)
  */
float cal_Iq_Id(float cur_b, float cur_c, float angle_el)
{
    /* Clarke transform */
    float I_alpha = -(cur_b + cur_c);
    float I_beta  = _1_SQRT3 * (cur_b - cur_c);

    /* Park transform */
    float s, c;
    arm_sin_cos_f32(angle_el * RAD_TO_DEG, &s, &c);
    float I_q = I_beta * c - I_alpha * s;
    return I_q;
}

/* ========================================================================== */
/*  Sensor alignment (from TinyFoc motor.c)                                   */
/* ========================================================================== */

/**
  * @brief  Align rotor to a known electrical angle by energizing a static
  *         voltage vector, then record the zero-angle offset.
  * @param  q_voltage  alignment voltage magnitude (2–4 V recommended)
  */
void foc_alignSensor(float q_voltage)
{
    char log_buf[128];
	
    /* Set flag so TIM callback does NOT overwrite PWM during alignment */
    alignment_in_progress = 1;

    /* Inject static voltage vector at 90° electrical to hold rotor */
    foc_forward(q_voltage, 0.0f, 0.0f);
    HAL_Delay(1000);

    /* ── Stop TIM2 encoder ISR: main thread takes exclusive encoder access ── */
    HAL_TIM_Base_Stop_IT(&htim2);

    /* Read encoder multiple times to let rotor settle */
    AS5047P_Sensor_Update(&AngleSensor); HAL_Delay(10);
    AS5047P_Sensor_Update(&AngleSensor); HAL_Delay(10);
    AS5047P_Sensor_Update(&AngleSensor); HAL_Delay(100);

    /* Compute zero-electric-angle via averaged readings */
    motor_control.zero_elec_angle = _calculate_zero_electric_angle();

    /* ── Prime DMA pipeline and restart TIM2 encoder ISR ── */
    AS5047P_DMA_StartRequest();
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);

    /* Reset both current-loop PIDs so they start from 0V smoothly */
    current_loop.integral_prev   = 0.0f;
    current_loop.output_prev     = 0.0f;
    current_loop.error_prev      = 0.0f;
    current_loop.timestamp_prev  = dwt_get_micros();

    id_current_loop.integral_prev   = 0.0f;
    id_current_loop.output_prev     = 0.0f;
    id_current_loop.error_prev      = 0.0f;
    id_current_loop.timestamp_prev  = dwt_get_micros();

    speed_loop.integral_prev   = 0.0f;
    speed_loop.output_prev     = 0.0f;
    speed_loop.error_prev      = 0.0f;
    speed_loop.timestamp_prev  = dwt_get_micros();

    /* Reset filter states so they don't carry stale values */
    motor_control.id_filter_state = 0.0f;
    motor_control.iq_filter_state = 0.0f;

    /* Enable current loop BEFORE clearing alignment flag —
     * this guarantees the very next ADC ISR takes over PWM,
     * eliminating the race where legacy SVPWM could run. */
    current_loop_enable = 1;

    /* Clear alignment flag — current loop is now in control */
    alignment_in_progress = 0;

    sprintf(log_buf, "[FOC] Zero elec angle = %.3f rad\r\n", motor_control.zero_elec_angle);
    UART2_SendString(log_buf);
    UART2_SendString("[FOC] Encoder Calibration Done!\r\n");
}

/* ========================================================================== */
/*  Closed-loop: current control                                               */
/* ========================================================================== */

/* Forward declarations for static helpers (defined after foc_current_loop)     */
static void foc_forward_cordic(float d, float q, float s_ff, float c_ff);
static void foc_speed_loop(void);
static void set_pwm_duty(float d_u, float d_v, float d_w);
static int  SVM(float alpha, float beta, float *tA, float *tB, float *tC);

/**
  * @brief  Speed (velocity) outer loop — 2 kHz, cascaded above current loop.
  *         Computes mechanical velocity from multi-turn total angle delta,
  *         applies LPF, runs PI controller, sets motor_control.set_torque.
  *
  *         Called from foc_current_loop() every 10th ADC ISR (SPEED_DECIMATION).
  */
static void foc_speed_loop(void)
{
    /* 1. Read current multi-turn total angle (volatile, atomic on M4) */
    float curr_total_angle = encoder_cache.total_angle_rad;

    /* 2. Re-init guard: capture angle, skip control this cycle.
     *    Triggered by motor_control_parm_init() or VOFA M=1.           */
    if (motor_control.spd_needs_init) {
        motor_control.spd_prev_angle = curr_total_angle;
        motor_control.vel_raw        = 0.0f;
        motor_control.vel_meas       = 0.0f;
        motor_control.spd_needs_init = 0;
        return;
    }

    /* 3. Compute velocity from delta angle (mechanical rad/s) */
    float delta   = curr_total_angle - motor_control.spd_prev_angle;
    motor_control.spd_prev_angle = curr_total_angle;
    float vel_raw = delta / SPEED_Ts;  /* SPEED_Ts = 0.0005s */

    /* 4. Low-pass filter velocity */
    float vel_filt = lowPassFilter(vel_raw, SPEED_LPF_ALPHA,
                                   &motor_control.vel_filter_state);

    /* Store for telemetry */
    motor_control.vel_raw  = vel_raw;
    motor_control.vel_meas = vel_filt;

    /* 5. PI control: error = setpoint - measured, corrected for dir */
    float error   = (float)motor_config.dir * (motor_control.set_speed - vel_filt);
    float iq_ref  = PIDController_Update(&speed_loop, error);

    /* 6. Clamp Iq reference to motor current limit.
     *    PIDController_Update already clamps to speed_loop.limit,
     *    but double-clamp is defense-in-depth. */
    if (iq_ref >  LIMIT_CURRENT) iq_ref =  LIMIT_CURRENT;
    if (iq_ref < -LIMIT_CURRENT) iq_ref = -LIMIT_CURRENT;

    motor_control.set_torque = iq_ref;
}

/**
  * @brief  Current (torque) loop — 20 kHz execution in ADC injection callback
  *
  *          Features:
  *            - Iq PI control (torque / speed / position outer loop)
  *            - Id PI control → keeps Id at 0 (MTPA for SPM motors)
  *            - Cross-coupling decoupling: Vd_ff = -ω·Lq·Iq
  *            - Back-EMF feedforward:      Vq_ff = +ω·(Ld·Id + ψm)
  *            - Single angle read per iteration → no drift between Park & iPark
  */
void foc_current_loop(void)
{
    /* ── 0. Speed loop decimation: run @ 2 kHz (every 10th ADC ISR) ── */
    {
        static uint8_t speed_cnt = 0;
        if (motor_control.mode == MOTOR_SPEED) {
            speed_cnt++;
            if (speed_cnt >= SPEED_DECIMATION) {
                speed_cnt = 0;
                foc_speed_loop();
            }
        } else {
            speed_cnt = 0;  /* hold at 0 in torque mode for clean transition */
        }
    }

    /* ── 1. Read angle with linear extrapolation between 10 kHz encoder updates ── */
    static uint32_t last_enc_update = 0;
    float angle_mech = encoder_cache.angle_raw;
    if (encoder_cache.update_count == last_enc_update) {
        /* Same encoder sample as last frame — extrapolate 50 µs (one 20 kHz period) */
        angle_mech += encoder_cache.velocity_rad_s * 10e-6f;
    }
    last_enc_update = encoder_cache.update_count;

    float angle_el = (float)(motor_config.dir * motor_config.pairs)
                     * angle_mech
                     - motor_control.zero_elec_angle;
    angle_el = _normalizeAngle(angle_el);
    float elec_vel = _electricalVelocity();  /* velocity still from TIM2 ISR cache */

    /* ── 0. Kick CORDIC: write angle_el Q31 → hw starts sin/cos for NEXT frame ── */
    {
        float a = angle_el;
        if (a >= PI) { a -= _2PI; }   /* [0,2π) → [-π,+π) for CORDIC */
        CORDIC->WDATA = (uint32_t)(int32_t)(a * CORDIC_Q31_PER_RAD);
    }

    /* ── 2. Clarke + Park: compute Id & Iq from B,C phase currents ── */
    float I_alpha = -(motor_control.IphB + motor_control.IphC);
    float I_beta  = _1_SQRT3 * (motor_control.IphB - motor_control.IphC);
    float s = cordic_sin_cache;   /* from previous-frame CORDIC */
    float c = cordic_cos_cache;
    float I_d_raw = I_alpha * c + I_beta * s;
    float I_q_raw = I_beta  * c - I_alpha * s;

    /* ── 3. Low-pass filter both axes (instance-based, no static clash) ── */
    motor_control.id_meas = lowPassFilter(I_d_raw, 0.05f, &motor_control.id_filter_state);
    motor_control.iq_meas = lowPassFilter(I_q_raw, 0.05f, &motor_control.iq_filter_state);

    /* ── 4. Cross-coupling + back-EMF feedforward ── */
    /*     Vd = Rs·Id + Ld·dId/dt - ω·Lq·Iq   →   Vd_ff = -ω·Lq·Iq            */
    /*     Vq = Rs·Iq + Lq·dIq/dt + ω·(Ld·Id+ψm) → Vq_ff = +ω·(Ld·Id+ψm)      */
    /* ── 4. 移除或修正前馈：低速/静止时关闭前馈，或对前馈电流进行强滤波 ── */
    float Vd_ff = 0.0f;
    float Vq_ff = 0.0f;
    


    /* ── 5. Iq error — source depends on control mode ── */
    float error_q = motor_control.set_torque - motor_control.iq_meas;

    /* ── 6. Id error — regulate to id_target (default 0 for SPM motors) ── */
    float error_d = motor_control.id_target - motor_control.id_meas;

    /* ── 7. Dead-band (Iq only — Id needs continuous regulation) ── */
    if (fabsf(error_q) <= 0.04f) {
        error_q = 0.0f;
    }
    /*
    if (fabsf(error_d) <= 0.00f) {
        error_d = 0.0f;
    }
    */
    /* ── 8. PI controllers ── */
    float Vd_pi = PIDController_Update(&id_current_loop, error_d);
    float Vq_pi = PIDController_Update(&current_loop,    error_q);

    /* ── 9. Combine PI output + feedforward ── */
    float Vd = Vd_pi + Vd_ff;
    float Vq = Vq_pi + Vq_ff;

    /* ── 10. Saturation — respect SVM linear modulation limit ── */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_limit   = mod_to_V * SQRT3_BY_2;   /* SVM inscribed circle       */
    float V_mag     = sqrtf(Vd * Vd + Vq * Vq);
    if (V_mag > V_limit) {
        float scale = V_limit / V_mag;
        Vd *= scale;
        Vq *= scale;
    }

    motor_control.id_set = Vd;   /* for debug telemetry                      */
    motor_control.iq_set = Vq;   /* for debug telemetry                      */

    /* ── 11. Inverse Park with cached sin/cos (same prev-frame angle).
     *        No feed-forward compensation — 50 µs pipeline delay dominates
     *        the 1.6·Ts intra-frame computational delay. PI compensates.  ── */
    foc_forward_cordic(Vd, Vq, cordic_sin_cache, cordic_cos_cache);
}

/* ========================================================================== */
/*  CORDIC-accelerated forward path — pre-computed sin/cos → SVM → PWM        */
/*  Replaces foc_forward() on the closed-loop hot path.                        */
/* ========================================================================== */
static void foc_forward_cordic(float d, float q, float s_ff, float c_ff)
{
    float d_u = 0.0f, d_v = 0.0f, d_w = 0.0f;

    /* Scale voltage commands to modulation index */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_to_mod  = 1.0f / mod_to_V;
    float mod_d     = V_to_mod * d;
    float mod_q     = V_to_mod * q;
    motor_control.mod_q = mod_q;

    /* Inverse Park — use caller-supplied sin/cos */
    float mod_alpha = mod_d * c_ff - mod_q * s_ff;
    float mod_beta  = mod_d * s_ff + mod_q * c_ff;

    /* SVM → duty cycles */
    SVM(mod_alpha, mod_beta, &d_u, &d_v, &d_w);

    /* Write to PWM registers */
    set_pwm_duty(d_u, d_v, d_w);
}

/* ========================================================================== */
/*  SVPWM forward path — d,q → duty cycles → PWM (from TinyFoc foc.c)         */
/* ========================================================================== */

/**
  * @brief  Write PWM duty cycles to TIM1 CCR registers
  */
static void set_pwm_duty(float d_u, float d_v, float d_w)
{
    d_u = _constrain(d_u, 0.0f, 0.9f);
    d_v = _constrain(d_v, 0.0f, 0.9f);
    d_w = _constrain(d_w, 0.0f, 0.9f);

    motor_control.du = d_u;
    motor_control.dv = d_v;
    motor_control.dw = d_w;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
}

/**
  * @brief  Space-Vector Modulation — convert α,β to 3-phase duty cycles
  * @return 0 on success, -1 if invalid sector
  */
static int SVM(float alpha, float beta, float *tA, float *tB, float *tC)
{
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            if (_1_SQRT3 * beta > alpha)
                Sextant = 2;
            else
                Sextant = 1;
        } else {
            if (-_1_SQRT3 * beta > alpha)
                Sextant = 3;
            else
                Sextant = 2;
        }
    } else {
        if (alpha >= 0.0f) {
            if (-_1_SQRT3 * beta > alpha)
                Sextant = 5;
            else
                Sextant = 6;
        } else {
            if (_1_SQRT3 * beta > alpha)
                Sextant = 4;
            else
                Sextant = 5;
        }
    }

    switch (Sextant) {
        case 1: {
            float t1 = alpha - _1_SQRT3 * beta;
            float t2 = _2_SQRT3 * beta;
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;
        case 2: {
            float t2 = alpha + _1_SQRT3 * beta;
            float t3 = -alpha + _1_SQRT3 * beta;
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;
        case 3: {
            float t3 = _2_SQRT3 * beta;
            float t4 = -alpha - _1_SQRT3 * beta;
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;
        case 4: {
            float t4 = -alpha + _1_SQRT3 * beta;
            float t5 = -_2_SQRT3 * beta;
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;
        case 5: {
            float t5 = -alpha - _1_SQRT3 * beta;
            float t6 = alpha - _1_SQRT3 * beta;
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;
        case 6: {
            float t6 = -_2_SQRT3 * beta;
            float t1 = alpha + _1_SQRT3 * beta;
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    int result_valid =
           *tA >= 0.0f && *tA <= 1.0f
        && *tB >= 0.0f && *tB <= 1.0f
        && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}

/**
  * @brief  Closed-loop FOC forward path: d,q voltage → inverse Park → SVM → PWM
  *
  * @param  d        d-axis voltage command
  * @param  q        q-axis voltage command
  * @param  angle_el electrical angle [rad]
  */
void foc_forward(float d, float q, float angle_el)
{
    float d_u = 0.0f, d_v = 0.0f, d_w = 0.0f;

    /* Scale voltage commands to modulation index.
     * Saturation already done in foc_current_loop() — single-point limit,
     * no redundant clamp here. */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_to_mod  = 1.0f / mod_to_V;
    float mod_d     = V_to_mod * d;
    float mod_q     = V_to_mod * q;
    motor_control.mod_q = mod_q;

    /* Inverse Park transform */
    float s, c;
    arm_sin_cos_f32(angle_el * RAD_TO_DEG, &s, &c);
    float mod_alpha = mod_d * c - mod_q * s;
    float mod_beta  = mod_d * s + mod_q * c;

    /* SVM → duty cycles */
    SVM(mod_alpha, mod_beta, &d_u, &d_v, &d_w);

    /* Write to PWM registers */
    set_pwm_duty(d_u, d_v, d_w);
}
