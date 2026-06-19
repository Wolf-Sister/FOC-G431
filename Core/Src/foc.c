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
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*  Global variables                                                          */
/* ========================================================================== */

/* Open-loop (kept from existing G431 code) */
OpenLoop_Ctrl_t motor_ctrl = {
    .Uq     = 0.3f,
    .Ud     = 0.0f,
    .Angle  = 0.0f,
    .Speed  = 0.0015f,
    .Period = 4250
};

/* Phase currents (dual-ADC captured) */
volatile Phase_Current_t motor_current = {0};

/* Current loop enable — set after calibration + alignment done */
volatile uint8_t current_loop_enable = 0;

/* Sensor alignment in progress — TIM callback must not overwrite PWM */
volatile uint8_t alignment_in_progress = 0;

/* Current-loop execution period (s) */
static const float current_meas_period = (1.0f / 5000.0f);  /* 5 kHz */

/* Motor config (from TinyFoc) */
motor_config_t motor_config = {
    .voltage_supply         = MOTOR_VBUS,
    .dir                    = -1,
    .pairs                  = 11,
    .pos_gain               = 0,
    .vel_gain               = 0,
    .vel_integrator_gain    = 0,
    .torque_gain            = 0,
    .torque_integrator_gain = 0,
};

/* Motor control state (from TinyFoc) */
motor_control_t motor_control = {
    .IphA              = 0.0f,
    .IphB              = 0.0f,
    .IphC              = 0.0f,
    .IphA_offset       = 0,
    .IphB_offset       = 0,
    .IphC_offset       = 0,
    .set_pos           = 0.0f,
    .set_vel           = 0.0f,
    .set_torque        = 0.0f,
    .mode              = MOTOR_TORQUE,
    .zero_elec_angle   = 0.0f,
    .pre_calibrated    = false,
    .encoder_updated   = false,
    .pos_abs           = 0,
    .iq_set            = 0.0f,
    .iq_meas           = 0.0f,
    .Iq_target         = 0.0f,
    .pos_target        = 0.0f,
    .vel_target        = 0.0f,
    .du                = 0.0f,
    .dv                = 0.0f,
    .dw                = 0.0f,
    .latest_ib_raw     = 0,
    .latest_ic_raw     = 0,
    .vel_lowpass_alpha = 0.0f,
    .mod_q             = 0.0f,
};

/* ========================================================================== */
/*  Existing SVPWM (kept for backward compat / open-loop testing)             */
/* ========================================================================== */

/**
  * @brief  7-segment SVPWM update — writes CCR1/2/3 for TIM1
  */
void SVPWM_Update(float Ud, float Uq, float angle, uint32_t period)
{
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
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

float FOC_GetSmoothAngle(void)
{
    static float filtered_angle = 0.0f;
    static uint8_t is_first = 1;

    uint16_t dma_raw = AS5047P_DMA_GetAngleCallback();

    if (dma_raw == 0xFFFFU) return filtered_angle;

    float current_angle = (float)dma_raw * (360.0f / 16384.0f);

    if (is_first) {
        filtered_angle = current_angle;
        is_first = 0;
        return filtered_angle;
    }

    float diff = current_angle - filtered_angle;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    filtered_angle += FOC_LPF_ALPHA * diff;

    if (filtered_angle >= 360.0f) filtered_angle -= 360.0f;
    if (filtered_angle <  0.0f)   filtered_angle += 360.0f;

    return filtered_angle;
}

void UART2_SendString(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
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
    motor_control.Iq_target  = 0.0f;
    motor_control.iq_meas    = 0.0f;
    motor_control.set_pos    = AS5047P_GetAngle(&AngleSensor);
    motor_control.set_vel    = 0.0f;
    motor_control.pos_target = 0.0f;
    motor_control.vel_target = 0.0f;
}

void set_motor_mode(Motor_Mode_e mode)
{
    motor_control.mode = mode;
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
    float elec_angle = (float)(motor_config.dir * motor_config.pairs)
                       * AS5047P_GetAngle(&AngleSensor)
                       - motor_control.zero_elec_angle;
    return _normalizeAngle(elec_angle);
}

/**
  * @brief  Get electrical angular velocity [rad/s]
  */
float _electricalVelocity(void)
{
    return (float)(motor_config.dir * motor_config.pairs)
           * AS5047P_GetVelocity(&AngleSensor);
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
    float I_q = I_beta * cosf(angle_el) - I_alpha * sinf(angle_el);
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

    /* Read encoder multiple times to let rotor settle */
    AS5047P_Sensor_Update(&AngleSensor); HAL_Delay(10);
    AS5047P_Sensor_Update(&AngleSensor); HAL_Delay(10);
    AS5047P_Sensor_Update(&AngleSensor); HAL_Delay(100);

    /* Compute zero-electric-angle via averaged readings */
    motor_control.zero_elec_angle = _calculate_zero_electric_angle();

    /* Reset current-loop PID so it starts from 0V smoothly */
    current_loop.integral_prev   = 0.0f;
    current_loop.output_prev     = 0.0f;
    current_loop.error_prev      = 0.0f;
    current_loop.timestamp_prev  = dwt_get_micros();

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
/*  Closed-loop: position / velocity / current (from TinyFoc motor.c)          */
/* ========================================================================== */

void foc_position_loop(void)
{
    float pos_error = motor_control.set_pos
                      - AS5047P_GetAccumulateAngle(&AngleSensor);

    /* Dead-band */
    if (fabsf(pos_error) < 0.01f) {
        motor_control.Iq_target = 0.0f;
    } else {
        motor_control.Iq_target = PIDController_Update(&angle_loop, pos_error);
    }
}

void foc_velocity_loop(void)
{
    float vel = AS5047P_GetVelocity(&AngleSensor);

    if (motor_control.mode == MOTOR_SPEED) {
        float error = motor_control.set_vel - vel;

        /* Dead-band with integrator clear */
        if (fabsf(motor_control.set_vel) < VEL_DEADBAND
            && fabsf(error) < VEL_DEADBAND) {
            motor_control.Iq_target      = 0.0f;
            vel_loop.integral_prev       = 0.0f;
            return;
        }

        motor_control.Iq_target  = PIDController_Update(&vel_loop, error);
        float ff_out             = KV_FF * motor_control.set_vel;
        motor_control.Iq_target += ff_out;
    }
}

/**
  * @brief  Current (torque) loop — 5 kHz execution in ADC injection callback
  */
void foc_current_loop(void)
{
    float error;

    /* Clarke + Park: compute Id & Iq from B,C phase currents */
    float angle_el = _electricalAngle();
    float I_alpha  = -(motor_control.IphB + motor_control.IphC);
    float I_beta   = _1_SQRT3 * (motor_control.IphB - motor_control.IphC);
    float I_d_raw  = I_alpha * cosf(angle_el) + I_beta * sinf(angle_el);
    float I_q_raw  = I_beta  * cosf(angle_el) - I_alpha * sinf(angle_el);

    motor_control.iq_meas = lowPassFilter(I_q_raw, 0.1f);
    motor_control.id_meas = I_d_raw;   /* raw Id for debug, no filter needed */

    /* Error source depends on mode */
    if (motor_control.mode == MOTOR_SPEED
        || motor_control.mode == MOTOR_POSITION) {
        error = motor_control.Iq_target - motor_control.iq_meas;
    } else {
        /* MOTOR_TORQUE: direct torque/current command */
        error = motor_control.set_torque - motor_control.iq_meas;
    }

    /* Dead-band to prevent dithering */
    if (fabsf(error) <= 0.00f) {
        error = 0.0f;
    }

    /* PID controller */
    motor_control.iq_set = PIDController_Update(&current_loop, error);

    /* Current dead-band */
    if (fabsf(motor_control.iq_set) < 0.02f) {
        motor_control.iq_set = 0.0f;
    }

    /* Saturation */
    motor_control.iq_set = _constrain(motor_control.iq_set,
                                      -LIMIT_CURRENT, LIMIT_CURRENT);

    /* Feed-forward angle compensation for computational delay */
    float t_delay   = 1.6f * current_meas_period;
    float elec_angle = _electricalAngle();
    float elec_vel   = _electricalVelocity();
    float angle_ff   = elec_angle + elec_vel * t_delay;

    /* SVPWM output */
    foc_forward(0.0f, motor_control.iq_set, angle_ff);
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

    __disable_irq();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * htim1.Instance->ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * htim1.Instance->ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * htim1.Instance->ARR);
    __enable_irq();
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

    /* Scale voltage commands to modulation index */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_to_mod  = 1.0f / mod_to_V;
    float mod_d     = V_to_mod * d;
    float mod_q     = V_to_mod * q;
    motor_control.mod_q = mod_q;

    /* Over-modulation clamping */
    float mod_scale = 0.80f * SQRT3_BY_2
                      * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scale < 1.0f) {
        mod_d *= mod_scale;
        mod_q *= mod_scale;
    }

    /* Inverse Park transform */
    float mod_alpha = mod_d * cosf(angle_el) - mod_q * sinf(angle_el);
    float mod_beta  = mod_d * sinf(angle_el) + mod_q * cosf(angle_el); // 将最后的 + 改为 -

    /* SVM → duty cycles */
    SVM(mod_alpha, mod_beta, &d_u, &d_v, &d_w);

    /* Write to PWM registers */
    set_pwm_duty(d_u, d_v, d_w);
}
