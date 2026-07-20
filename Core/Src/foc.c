/**
  ******************************************************************************
  * @file    foc.c
  * @brief   FOC 电机控制 — SVPWM、电流环、传感器对齐
  *
  *          将 TinyFoc 的 motor.c + foc.c 合并到现有 G431 模块中
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
/*  全局变量                                                                   */
/* ========================================================================== */

/* 相电流（双 ADC 采集） */
volatile Phase_Current_t motor_current = {0};

/* 电流环使能 — 校准 + 对齐完成后设置 */
volatile uint8_t current_loop_enable = 0;

/* Set only after all motor-control state has been initialized. */
volatile uint8_t motor_ready = 0;

/* 传感器对齐进行中 — TIM 回调不得覆盖 PWM */
volatile uint8_t alignment_in_progress = 0;

/* 电机配置 (来自 TinyFoc) */
motor_config_t motor_config = {
    .voltage_supply         = MOTOR_VBUS,
    .dir                    = -1,
    .pairs                  = 11,
    .iq_p_gain              = IQ_CURRENT_KP_DEFAULT,
    .iq_i_gain              = IQ_CURRENT_KI_DEFAULT,
    .id_p_gain              = ID_CURRENT_KP_DEFAULT,
    .id_i_gain              = ID_CURRENT_KI_DEFAULT,
    .spd_p_gain             = SPEED_KP_DEFAULT,
    .spd_i_gain             = SPEED_KI_DEFAULT,
    .spd_p_low_speed       = SPEED_KP_LOW_SPEED_DEFAULT,
    .spd_p_high_speed      = SPEED_KP_HIGH_SPEED_DEFAULT,
    .spd_gain_schedule     = SPEED_GAIN_SCHEDULE_DEFAULT,
    .pos_p_gain             = POSITION_KP_DEFAULT,
    .current_voltage_limit  = CURRENT_VOLTAGE_LIMIT_DEFAULT,
    .speed_current_limit    = SPEED_CURRENT_LIMIT_DEFAULT,
    .pos_speed_limit        = POS_SPEED_LIMIT_DEFAULT,
    .pos_accel_limit        = POS_ACCEL_LIMIT_DEFAULT,
};

/* 电机控制状态 (来自 TinyFoc) */
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
    .set_position      = 0.0f,
    .pos_meas          = 0.0f,
    .vel_meas          = 0.0f,
    .vel_raw           = 0.0f,
    .vel_filter_state  = 0.0f,
    .spd_kp_active     = SPEED_KP_LOW_SPEED_DEFAULT,
    .spd_gain_region   = SPEED_GAIN_REGION_LOW_SPEED,
    .mod_q             = 0.0f,
    .mod_d             = 0.0f,
    .du                = 0.0f,
    .dv                = 0.0f,
    .dw                = 0.0f,
    .latest_ib_raw     = 0,
    .latest_ic_raw     = 0,
};

/* CORDIC sin/cos 缓存 — CORDIC ISR 写入, foc_current_loop() 读取              */
volatile float cordic_sin_cache = 0.0f;
volatile float cordic_cos_cache = 1.0f;  /* cos(0)=1 安全初始值                  */

/* ========================================================================== */
/*  现有 SVPWM（保留用于向后兼容 / 开环测试）                                  */
/* ========================================================================== */

/**
  * @brief  11 段 SVPWM 更新 — 写入 TIM1 CCR1/2/3
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
/*  校准 & 角度滤波器（保留）                                                   */
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
    /* 等待上一次 DMA 传输完成（DMA_NORMAL 模式：
     * TC 中断服务函数会将 gState 恢复为 READY） */
    while (huart2.gState != HAL_UART_STATE_READY) {}
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)str, strlen(str));
}

/* ========================================================================== */
/*  相电流同步 — 将 ADC 采集电流复制到 motor_control                           */
/* ========================================================================== */

/**
  * @brief  将相电流从双 ADC 缓冲区同步到 motor_control 结构体
  *         在 ADC 注入回调中、电流环之前调用
  */
void foc_sync_phase_currents(void)
{
    motor_control.IphA = motor_current.I_A;
    motor_control.IphB = motor_current.I_B;
    motor_control.IphC = motor_current.I_C;
}

/* ========================================================================== */
/*  电机参数初始化                                                             */
/* ========================================================================== */

void motor_control_parm_init(void)
{
    encoder_cache_t encoder = {0};
    (void)AS5047P_EncoderCache_Read(&encoder);

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
    motor_control.spd_needs_init   = 1;
    motor_control.set_position     = encoder.total_angle_rad;
    motor_control.pos_meas         = encoder.total_angle_rad;
}

static void foc_clamp_pid_state(struct PIDController *pid, float limit)
{
    if (pid->integral_prev >  limit) pid->integral_prev =  limit;
    if (pid->integral_prev < -limit) pid->integral_prev = -limit;
    if (pid->output_prev >  limit) pid->output_prev =  limit;
    if (pid->output_prev < -limit) pid->output_prev = -limit;
}

void foc_set_loop_limits(float current_voltage_limit,
                         float speed_current_limit,
                         float position_speed_limit)
{
    float hardware_voltage_limit = motor_config.voltage_supply / SQRT3;

    if (!(current_voltage_limit > 0.0f)) {
        current_voltage_limit = motor_config.current_voltage_limit;
    }
    if (!(speed_current_limit > 0.0f)) {
        speed_current_limit = motor_config.speed_current_limit;
    }
    if (!(position_speed_limit > 0.0f)) {
        position_speed_limit = motor_config.pos_speed_limit;
    }

    if (current_voltage_limit > hardware_voltage_limit) {
        current_voltage_limit = hardware_voltage_limit;
    }
    if (speed_current_limit > ABSOLUTE_CURRENT_LIMIT) {
        speed_current_limit = ABSOLUTE_CURRENT_LIMIT;
    }
    if (position_speed_limit > POS_SPEED_LIMIT_MAX) {
        position_speed_limit = POS_SPEED_LIMIT_MAX;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    motor_config.current_voltage_limit = current_voltage_limit;
    current_loop.limit = current_voltage_limit;
    id_current_loop.limit = current_voltage_limit;
    foc_clamp_pid_state(&current_loop, current_voltage_limit);
    foc_clamp_pid_state(&id_current_loop, current_voltage_limit);

    motor_config.speed_current_limit = speed_current_limit;
    speed_loop.limit = speed_current_limit;
    foc_clamp_pid_state(&speed_loop, speed_current_limit);
    if (motor_control.set_torque > speed_current_limit) {
        motor_control.set_torque = speed_current_limit;
    }
    if (motor_control.set_torque < -speed_current_limit) {
        motor_control.set_torque = -speed_current_limit;
    }
    if (motor_control.id_target > speed_current_limit) {
        motor_control.id_target = speed_current_limit;
    }
    if (motor_control.id_target < -speed_current_limit) {
        motor_control.id_target = -speed_current_limit;
    }

    motor_config.pos_speed_limit = position_speed_limit;

    __set_PRIMASK(primask);
}

/* ========================================================================== */
/*  电气角度辅助函数 (来自 TinyFoc utils.c)                                    */
/* ========================================================================== */

/**
  * @brief  在传感器对齐过程中计算零电角偏移量
  */
float _calculate_zero_electric_angle(void)
{
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    uint8_t valid_samples = 0U;

    while (valid_samples < 10U) {
        if (AS5047P_Sensor_Update(&AngleSensor)) {
            float angle = AS5047P_GetAngle(&AngleSensor);
            float s;
            float c;
            arm_sin_cos_f32(angle * RAD_TO_DEG, &s, &c);
            sum_sin += s;
            sum_cos += c;
            valid_samples++;
        }
        HAL_Delay(1);
    }

    float mech_angle = atan2f(sum_sin, sum_cos);
    if (mech_angle < 0.0f) mech_angle += _2PI;

    float raw_elec_angle = (float)(motor_config.dir * motor_config.pairs) * mech_angle;
    return _normalizeAngle(raw_elec_angle);
}

/**
  * @brief  获取瞬时电角度 [0, 2π)
  */
float _electricalAngle(void)
{
    encoder_cache_t encoder = {0};
    (void)AS5047P_EncoderCache_Read(&encoder);
    float mech_angle = encoder.angle_raw;
    float elec_angle = (float)(motor_config.dir * motor_config.pairs)
                       * mech_angle
                       - motor_control.zero_elec_angle;
    return _normalizeAngle(elec_angle);
}

/**
  * @brief  获取电角速度 [rad/s]
  */
float _electricalVelocity(void)
{
    encoder_cache_t encoder = {0};
    (void)AS5047P_EncoderCache_Read(&encoder);
    float mech_vel = encoder.velocity_rad_s;
    return (float)(motor_config.dir * motor_config.pairs) * mech_vel;
}

/* ========================================================================== */
/*  Clarke + Park 变换 (来自 TinyFoc motor.c)                                  */
/* ========================================================================== */

/**
  * @brief  从 B、C 相电流通过 Clarke + Park 变换计算 Iq
  * @param  cur_b    B 相电流 (A)
  * @param  cur_c    C 相电流 (A)
  * @param  angle_el 电角度 [rad]
  * @return Iq (产生转矩的电流分量)
  */
float cal_Iq_Id(float cur_b, float cur_c, float angle_el)
{
    /* Clarke 变换 */
    float I_alpha = -(cur_b + cur_c);
    float I_beta  = _1_SQRT3 * (cur_b - cur_c);

    /* Park 变换 */
    float s, c;
    arm_sin_cos_f32(angle_el * RAD_TO_DEG, &s, &c);
    float I_q = I_beta * c - I_alpha * s;
    return I_q;
}

/* Return the largest absolute phase-current value during alignment. */
static float foc_align_max_phase_current(void)
{
    float max_current = fabsf(motor_current.I_A);
    float phase_current = fabsf(motor_current.I_B);

    if (phase_current > max_current) {
        max_current = phase_current;
    }

    phase_current = fabsf(motor_current.I_C);
    if (phase_current > max_current) {
        max_current = phase_current;
    }

    return max_current;
}

/* Delay in 1 ms steps and reject sustained alignment overcurrent. */
static HAL_StatusTypeDef foc_align_wait(uint32_t delay_ms,
                                        uint32_t *overcurrent_ms)
{
    uint32_t elapsed_ms;

    for (elapsed_ms = 0U; elapsed_ms < delay_ms; elapsed_ms++) {
        HAL_Delay(1U);

        if (foc_align_max_phase_current() > FOC_ALIGN_CURRENT_LIMIT_A) {
            (*overcurrent_ms)++;
            if (*overcurrent_ms >= FOC_ALIGN_OVERCURRENT_MS) {
                return HAL_ERROR;
            }
        } else {
            *overcurrent_ms = 0U;
        }
    }

    return HAL_OK;
}

/* Change the static alignment voltage smoothly in 1 ms steps. */
static HAL_StatusTypeDef foc_align_ramp(float start_voltage,
                                        float end_voltage,
                                        uint32_t ramp_ms,
                                        uint32_t *overcurrent_ms)
{
    uint32_t step;

    if (ramp_ms == 0U) {
        foc_forward(end_voltage, 0.0f, 0.0f);
        return foc_align_wait(1U, overcurrent_ms);
    }

    for (step = 1U; step <= ramp_ms; step++) {
        float ratio = (float)step / (float)ramp_ms;
        float voltage = start_voltage + ((end_voltage - start_voltage) * ratio);

        foc_forward(voltage, 0.0f, 0.0f);
        if (foc_align_wait(1U, overcurrent_ms) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  传感器对齐 (来自 TinyFoc motor.c)                                          */
/* ========================================================================== */

/**
  * @brief  通过注入静态电压矢量将转子对齐到已知电角度，记录零角偏移量
  * @retval HAL_OK 对齐完成；HAL_ERROR 对齐阶段持续过流
  */
HAL_StatusTypeDef foc_alignSensor(void)
{
    char log_buf[128];
    uint32_t overcurrent_ms = 0U;
    uint8_t encoder_timer_stopped = 0U;
	
    /* 设置标志，防止 TIM 回调在对齐期间覆盖 PWM */
    alignment_in_progress = 1;

    /* 从零开始渐升静态电压矢量，避免阶跃激发机械啸叫。 */
    foc_forward(0.0f, 0.0f, 0.0f);
    if (foc_align_ramp(0.0f,
                       FOC_ALIGN_VOLTAGE_V,
                       FOC_ALIGN_RAMP_UP_MS,
                       &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }

    if (foc_align_wait(FOC_ALIGN_HOLD_MS, &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }

    /* Lower the holding torque before sampling the encoder zero point. */
    if (foc_align_ramp(FOC_ALIGN_VOLTAGE_V,
                       FOC_ALIGN_HOLD_VOLTAGE_V,
                       FOC_ALIGN_RAMP_DOWN_MS,
                       &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }

    /* ── 停止 TIM2 编码器 ISR：主线程独占编码器访问 ── */
    HAL_TIM_Base_Stop_IT(&htim2);
    encoder_timer_stopped = 1U;

    /* 多次读取编码器，等待转子稳定 */
    AS5047P_Sensor_Update(&AngleSensor);
    if (foc_align_wait(10U, &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }
    AS5047P_Sensor_Update(&AngleSensor);
    if (foc_align_wait(10U, &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }
    AS5047P_Sensor_Update(&AngleSensor);
    if (foc_align_wait(100U, &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }

    /* 通过平均多次读数计算零电角 */
    motor_control.zero_elec_angle = _calculate_zero_electric_angle();

    /* Remove the holding voltage smoothly after zero-angle sampling. */
    if (foc_align_ramp(FOC_ALIGN_HOLD_VOLTAGE_V,
                       0.0f,
                       FOC_ALIGN_RAMP_DOWN_MS,
                       &overcurrent_ms) != HAL_OK) {
        goto alignment_failed;
    }
    foc_forward(0.0f, 0.0f, 0.0f);

    /* ── 启动 DMA 流水线并重启 TIM2 编码器 ISR ── */
    AS5047P_DMA_StartRequest();
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
    encoder_timer_stopped = 0U;

    /* 重置两个电流环 PID，使其从 0V 平滑启动 */
    current_loop.integral_prev   = 0.0f;
    current_loop.output_prev     = 0.0f;
    current_loop.error_prev      = 0.0f;
    current_loop.timestamp_prev_cycles  = dwt_get_cycles();

    id_current_loop.integral_prev   = 0.0f;
    id_current_loop.output_prev     = 0.0f;
    id_current_loop.error_prev      = 0.0f;
    id_current_loop.timestamp_prev_cycles  = dwt_get_cycles();

    speed_loop.integral_prev   = 0.0f;
    speed_loop.output_prev     = 0.0f;
    speed_loop.error_prev      = 0.0f;
    speed_loop.timestamp_prev_cycles  = dwt_get_cycles();

    /* 重置滤波器状态，避免残余值干扰 */
    motor_control.id_filter_state = 0.0f;
    motor_control.iq_filter_state = 0.0f;

    /* Alignment is complete. main() initializes all control state before it
     * enables the current loop, so the ADC ISR cannot race startup writes. */
    alignment_in_progress = 0;

    sprintf(log_buf, "[FOC] Zero elec angle = %.3f rad\r\n", motor_control.zero_elec_angle);
    UART2_SendString(log_buf);
    UART2_SendString("[FOC] Encoder Calibration Done!\r\n");
    return HAL_OK;

alignment_failed:
    /* Remove voltage before main() disables the gate driver. */
    foc_forward(0.0f, 0.0f, 0.0f);
    if (encoder_timer_stopped != 0U) {
        AS5047P_DMA_StartRequest();
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim2);
    }
    alignment_in_progress = 0;
    UART2_SendString("[FOC] Alignment aborted: phase current limit exceeded!\r\n");
    return HAL_ERROR;
}

/* ========================================================================== */
/*  闭环: 电流控制                                                             */
/* ========================================================================== */

/* 静态辅助函数前向声明（在 foc_current_loop 之后定义）                          */
static void foc_forward_cordic(float d, float q, float s_ff, float c_ff);
static void foc_cordic_sin_cos_current(float angle_el, float *s, float *c);
static void foc_speed_loop(const encoder_cache_t *encoder);
static void set_pwm_duty(float d_u, float d_v, float d_w);
static int  SVM(float alpha, float beta, float *tA, float *tB, float *tC);

/**
  * @brief  Compute current-frame CORDIC sin/cos; use the last valid result on timeout
  */
static void foc_cordic_sin_cos_current(float angle_el, float *s, float *c)
{
    float a = angle_el;
    uint32_t poll_count = FOC_CORDIC_POLL_LIMIT;
    /* Drain a late result left by a previous timeout before starting a new frame. */
    if ((CORDIC->CSR & CORDIC_CSR_RRDY) != 0U) {
        (void)CORDIC->RDATA;
        (void)CORDIC->RDATA;
    }


    if (a >= PI) {
        a -= _2PI;  /* [0, 2pi) -> [-pi, +pi) */
    }

    CORDIC->WDATA = (uint32_t)(int32_t)(a * CORDIC_Q31_PER_RAD);
    while (((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) && (poll_count > 0U)) {
        poll_count--;
    }

    if ((CORDIC->CSR & CORDIC_CSR_RRDY) != 0U) {
        int32_t cos_q31 = (int32_t)CORDIC->RDATA;
        int32_t sin_q31 = (int32_t)CORDIC->RDATA;

        *c = (float)cos_q31 / 2147483648.0f;
        *s = (float)sin_q31 / 2147483648.0f;
        cordic_cos_cache = *c;
        cordic_sin_cache = *s;
    } else {
        *c = cordic_cos_cache;
        *s = cordic_sin_cache;
    }
}

/**
  * @brief  位置外环 — 1 kHz，在 TIM3 ISR 中调用
  *         P 位置控制产生速度目标，并受最大速度、加速度和
  *         剩余制动距离共同约束，避免大位置阶跃直接冲击速度环。
  *         规划后的 motor_control.set_speed 供 2 kHz 速度环使用。
  *
  *         读取 TIM2 ISR (P=1) 发布的编码器一致性快照
  *         写入 motor_control.set_speed (ADC ISR 读取者, P=0)
  *         如果 mode != MOTOR_POSITION 则立即返回
  */
void foc_position_loop(void)
{
    if (motor_control.mode != MOTOR_POSITION) return;

    /* 1. 读取当前多圈位置 */
    encoder_cache_t encoder = {0};
    (void)AS5047P_EncoderCache_Read(&encoder);
    float curr_pos = encoder.total_angle_rad;

    /* 2. P 位置控制给出期望速度 */
    float pos_error = motor_control.set_position - curr_pos;
    float speed_cmd = pos_error * motor_config.pos_p_gain;

    /* 3. 用户指定的硬速度上限 */
    float speed_limit = motor_config.pos_speed_limit;
    if (speed_cmd >  speed_limit) speed_cmd =  speed_limit;
    if (speed_cmd < -speed_limit) speed_cmd = -speed_limit;

    /* 4. 按剩余距离计算可制动速度，形成梯形/三角形速度轨迹 */
    float accel_limit = motor_config.pos_accel_limit;
    if (!(accel_limit > 0.0f)) accel_limit = POS_ACCEL_LIMIT_DEFAULT;
    float braking_speed = sqrtf(2.0f * accel_limit * fabsf(pos_error));
    if (speed_cmd >  braking_speed) speed_cmd =  braking_speed;
    if (speed_cmd < -braking_speed) speed_cmd = -braking_speed;

    /* 5. 1 kHz 速度指令斜率限制，避免速度目标瞬时跳变 */
    float max_speed_delta = accel_limit * POSITION_LOOP_DT_S;
    float planned_speed = motor_control.set_speed;
    if (speed_cmd > planned_speed + max_speed_delta) {
        planned_speed += max_speed_delta;
    } else if (speed_cmd < planned_speed - max_speed_delta) {
        planned_speed -= max_speed_delta;
    } else {
        planned_speed = speed_cmd;
    }

    /* 6. 馈入速度环 */
    motor_control.set_speed = planned_speed;
    motor_control.pos_meas  = curr_pos;
}

static float foc_select_speed_kp(float speed_ref)
{
    if (motor_config.spd_gain_schedule == 0U) {
        return motor_config.spd_p_gain;
    }

    float abs_speed_ref = fabsf(speed_ref);
    if (motor_control.spd_gain_region == SPEED_GAIN_REGION_LOW_SPEED) {
        if (abs_speed_ref >= SPEED_KP_SWITCH_UP_RAD_S) {
            motor_control.spd_gain_region = SPEED_GAIN_REGION_HIGH_SPEED;
        }
    } else if (abs_speed_ref <= SPEED_KP_SWITCH_DOWN_RAD_S) {
        motor_control.spd_gain_region = SPEED_GAIN_REGION_LOW_SPEED;
    }

    return (motor_control.spd_gain_region == SPEED_GAIN_REGION_LOW_SPEED)
         ? motor_config.spd_p_low_speed
         : motor_config.spd_p_high_speed;
}

static void foc_set_speed_kp_bumpless(float new_kp, float error)
{
    float old_kp = speed_loop.P;
    if (fabsf(new_kp - old_kp) < 0.000001f) {
        motor_control.spd_kp_active = old_kp;
        return;
    }

    float integral = speed_loop.integral_prev + (old_kp - new_kp) * error;
    if (integral >  speed_loop.limit) integral =  speed_loop.limit;
    if (integral < -speed_loop.limit) integral = -speed_loop.limit;

    speed_loop.integral_prev = integral;
    speed_loop.P = new_kp;
    motor_control.spd_kp_active = new_kp;
}

static float foc_initial_speed_kp(float speed_ref)
{
    motor_control.spd_gain_region =
        (fabsf(speed_ref) >= SPEED_KP_SWITCH_UP_RAD_S)
        ? SPEED_GAIN_REGION_HIGH_SPEED : SPEED_GAIN_REGION_LOW_SPEED;
    return foc_select_speed_kp(speed_ref);
}

/**
  * @brief  速度外环 — 2 kHz，级联在电流环之上
  *         滤波编码器的 4 ms 整数计数窗口速度，运行
  *         PI 控制器，并设置 motor_control.set_torque
  *
  *         在 foc_current_loop() 中每 10 次 ADC ISR (SPEED_DECIMATION) 调用一次
  */
static void foc_speed_loop(const encoder_cache_t *encoder)
{
    /* 1. 重新初始化保护：在控制 ISR 中重置动态状态 */
    if (motor_control.spd_needs_init) {
        motor_control.vel_filter_state = 0.0f;
        motor_control.vel_raw          = 0.0f;
        motor_control.vel_meas         = 0.0f;
        speed_loop.P = foc_initial_speed_kp(motor_control.set_speed);
        speed_loop.I = motor_config.spd_i_gain;
        motor_control.spd_kp_active = speed_loop.P;
        speed_pid_reset();
        motor_control.spd_needs_init   = 0;
        return;
    }

    /* 2. 使用编码器的 4 ms 整数计数滑动窗口速度 */
    float vel_raw = encoder->velocity_rad_s;

    /* 3. 在 2 kHz 速度环频率下应用约 17 Hz 一阶低通滤波器 */
    float vel_filt = lowPassFilter(vel_raw, SPEED_LPF_ALPHA,
                                   &motor_control.vel_filter_state);

    /* 存储用于遥测 */
    motor_control.vel_raw  = vel_raw;
    motor_control.vel_meas = vel_filt;

    /* 4. PI 控制：误差 = 目标 - 测量值，按方向修正 */
    float error = (float)motor_config.dir * (motor_control.set_speed - vel_filt);
    float scheduled_kp = foc_select_speed_kp(motor_control.set_speed);
    foc_set_speed_kp_bumpless(scheduled_kp, error);
    speed_loop.I = motor_config.spd_i_gain;

    float iq_ref = PIDController_Update(&speed_loop, error);

    /* 5. 将 Iq 参考钳位到电机电流限幅值
     *    PIDController_Update 已经钳位到 speed_loop.limit，
     *    此处二次钳位为深度防御 */
    float current_limit = motor_config.speed_current_limit;
    if (iq_ref >  current_limit) iq_ref =  current_limit;
    if (iq_ref < -current_limit) iq_ref = -current_limit;

    motor_control.set_torque = iq_ref;
}

/**
  * @brief  电流（转矩）环 — 20 kHz，在 ADC 注入回调中执行
  *
  *          功能:
  *            - Iq PI 控制（转矩 / 速度 / 位置外环）
  *            - Id PI 控制 → 保持 Id = 0（SPM 电机 MTPA）
  *            - 交叉耦合解耦: Vd_ff = -ω·Lq·Iq
  *            - 反电动势前馈:  Vq_ff = +ω·(Ld·Id + ψm)
  *            - 每次迭代单次角度读取 → Park 和 iPark 之间无漂移
  */
void foc_current_loop(void)
{
    encoder_cache_t encoder = {0};
    (void)AS5047P_EncoderCache_Read(&encoder);

    /* ── 0. 速度环降采样: 每 10 次 ADC ISR 运行一次 @ 2 kHz ── */
    {
        static uint8_t speed_cnt = 0;
        if (motor_control.mode == MOTOR_SPEED || motor_control.mode == MOTOR_POSITION) {
            speed_cnt++;
            if (speed_cnt >= SPEED_DECIMATION) {
                speed_cnt = 0;
                foc_speed_loop(&encoder);
            }
        } else {
            speed_cnt = 0;  /* 转矩模式下保持在 0，确保干净的模式切换 */
        }
    }

    /* ── 1. 按 DMA 实际采样时刻，将编码器角度预测到当前电流帧 ── */
    float angle_mech = encoder.angle_raw;
    uint32_t angle_age_cycles = dwt_get_cycles() - encoder.sample_cycle;
    float angle_age_s = dwt_cycles_to_seconds(angle_age_cycles);
    if (angle_age_s > FOC_ENCODER_PREDICTION_MAX_S) {
        angle_age_s = FOC_ENCODER_PREDICTION_MAX_S;
    }
    angle_mech += encoder.velocity_rad_s * angle_age_s;

    float angle_el = (float)(motor_config.dir * motor_config.pairs)
                     * angle_mech
                     - motor_control.zero_elec_angle;
    angle_el = _normalizeAngle(angle_el);
    float elec_vel = (float)(motor_config.dir * motor_config.pairs)
                     * encoder.velocity_rad_s;

    /* ── 0. 当前帧 CORDIC: 写入 angle_el 并立即短轮询读取 sin/cos ── */
    float s;
    float c;
    foc_cordic_sin_cos_current(angle_el, &s, &c);

    /* ── 2. Clarke + Park: 从 B、C 相电流计算 Id & Iq ── */
    float I_alpha = -(motor_control.IphB + motor_control.IphC);
    float I_beta  = _1_SQRT3 * (motor_control.IphB - motor_control.IphC);
    float I_d_raw = I_alpha * c + I_beta * s;
    float I_q_raw = I_beta  * c - I_alpha * s;

    /* ── 3. 双轴低通滤波, alpha=0.05 → fc≈163Hz @ 20kHz ── */
    motor_control.id_meas = lowPassFilter(I_d_raw, 0.05f, &motor_control.id_filter_state);
    motor_control.iq_meas = lowPassFilter(I_q_raw, 0.05f, &motor_control.iq_filter_state);
		
    /* ── 4. 交叉耦合 + 反电动势前馈 ── */
    /*     Vd = Rs·Id + Ld·dId/dt - ω·Lq·Iq   →   Vd_ff = -ω·Lq·Iq            */
    /*     Vq = Rs·Iq + Lq·dIq/dt + ω·(Ld·Id+ψm) → Vq_ff = +ω·(Ld·Id+ψm)      */
    float Vd_ff = 0.0f;
    float Vq_ff = 0.0f;
#if FOC_FEEDFORWARD_ENABLE
    float ff_blend = 0.0f;
    float abs_elec_vel = fabsf(elec_vel);
    if (abs_elec_vel >= FF_FULL_ELEC_RAD_S) {
        ff_blend = 1.0f;
    } else if (abs_elec_vel > FF_START_ELEC_RAD_S &&
               FF_FULL_ELEC_RAD_S > FF_START_ELEC_RAD_S) {
        float x = (abs_elec_vel - FF_START_ELEC_RAD_S)
                / (FF_FULL_ELEC_RAD_S - FF_START_ELEC_RAD_S);
        ff_blend = x * x * (3.0f - 2.0f * x);
    }
    Vd_ff = ff_blend * (-elec_vel * MOTOR_Lq * motor_control.iq_meas);
    Vq_ff = ff_blend * ( elec_vel * (MOTOR_Ld * motor_control.id_meas + MOTOR_FLUX));
#endif

    /* ── 5. 调节前限制 dq 电流参考矢量 ── */
    float iq_target = motor_control.set_torque;
    float id_target = motor_control.id_target;
    float current_limit = motor_config.speed_current_limit;
    float current_ref_sq = iq_target * iq_target + id_target * id_target;
    float current_limit_sq = current_limit * current_limit;
    if (current_ref_sq > current_limit_sq && current_ref_sq > 0.0f) {
        float scale = current_limit / sqrtf(current_ref_sq);
        iq_target *= scale;
        id_target *= scale;
    }

    float error_q = iq_target - motor_control.iq_meas;

    /* ── 6. Id 误差 — 调节到限幅后的 id_target ── */
    float error_d = id_target - motor_control.id_meas;

    /* ── 7. 死区 (仅 Iq — Id 需要连续调节) ── */
		/*
    if (fabsf(error_q) <= 0.00f) {
        error_q = 0.0f;
    }
		*/
    /*
    if (fabsf(error_d) <= 0.00f) {
        error_d = 0.0f;
    }
    */
    /* ── 8. PI 控制器 ── */
    float Vd_pi = PIDController_Update(&id_current_loop, error_d);
    float Vq_pi = PIDController_Update(&current_loop,    error_q);

    /* ── 9. 合并 PI 输出 + 前馈 ── */
    float Vd = Vd_pi + Vd_ff;
    float Vq = Vq_pi + Vq_ff;
    float Vd_requested = Vd;
    float Vq_requested = Vq;

    /* ── 10. 饱和处理 — 遵守 SVM 线性调制限幅 ── */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float hardware_V_limit = mod_to_V * SQRT3_BY_2;
    float V_limit = motor_config.current_voltage_limit;
    if (V_limit > hardware_V_limit || V_limit <= 0.0f) {
        V_limit = hardware_V_limit;
    }
    float V_mag     = sqrtf(Vd * Vd + Vq * Vq);
    if (V_mag > V_limit) {
        float scale = V_limit / V_mag;
        Vd *= scale;
        Vq *= scale;
    }

    motor_control.id_set = Vd;   /* 用于调试遥测                             */
    motor_control.iq_set = Vq;   /* 用于调试遥测                             */

    /* Track final coupled dq saturation, including feed-forward voltage. */
    if (Vd != Vd_requested || Vq != Vq_requested) {
        PIDController_ApplyTracking(&id_current_loop, Vd - Vd_requested,
                                    CURRENT_VECTOR_AW_GAIN_DEFAULT);
        PIDController_ApplyTracking(&current_loop, Vq - Vq_requested,
                                    CURRENT_VECTOR_AW_GAIN_DEFAULT);
    }

    /* ── 11. 使用同一当前帧的 sin/cos 做逆 Park 变换 ── */
    foc_forward_cordic(Vd, Vq, s, c);
}

/* ========================================================================== */
/*  CORDIC 加速前向通道 — 预计算 sin/cos → SVM → PWM                          */
/*  在闭环热路径上替代 foc_forward()                                           */
/* ========================================================================== */
static void foc_forward_cordic(float d, float q, float s_ff, float c_ff)
{
    float d_u = 0.0f, d_v = 0.0f, d_w = 0.0f;

    /* 将电压指令缩放为调制指数 */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_to_mod  = 1.0f / mod_to_V;
    float mod_d     = V_to_mod * d;
    float mod_q     = V_to_mod * q;
    motor_control.mod_q = mod_q;

    motor_control.mod_d = mod_d;
    /* 逆 Park 变换 — 使用调用者提供的 sin/cos */
    float mod_alpha = mod_d * c_ff - mod_q * s_ff;
    float mod_beta  = mod_d * s_ff + mod_q * c_ff;

    /* SVM → 占空比 */
    SVM(mod_alpha, mod_beta, &d_u, &d_v, &d_w);

    /* 写入 PWM 寄存器 */
    set_pwm_duty(d_u, d_v, d_w);
}

/* ========================================================================== */
/*  SVPWM 前向通道 — d,q → 占空比 → PWM (来自 TinyFoc foc.c)                  */
/* ========================================================================== */

/**
  * @brief  将 PWM 占空比写入 TIM1 CCR 寄存器
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
  * @brief  空间矢量调制 — 将 α,β 转换为三相占空比
  * @return 成功返回 0，无效扇区返回 -1
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
  * @brief  闭环 FOC 前向通道: d,q 电压 → 逆 Park 变换 → SVM → PWM
  *
  * @param  d        d 轴电压指令
  * @param  q        q 轴电压指令
  * @param  angle_el 电角度 [rad]
  */
void foc_forward(float d, float q, float angle_el)
{
    float d_u = 0.0f, d_v = 0.0f, d_w = 0.0f;

    /* 将电压指令缩放为调制指数
     * 饱和处理已在 foc_current_loop() 中完成 — 单点限幅，
     * 此处无需重复钳位 */
    float mod_to_V  = (2.0f / 3.0f) * motor_config.voltage_supply;
    float V_to_mod  = 1.0f / mod_to_V;
    float mod_d     = V_to_mod * d;
    float mod_q     = V_to_mod * q;
    motor_control.mod_q = mod_q;

    motor_control.mod_d = mod_d;
    /* 逆 Park 变换 */
    float s, c;
    arm_sin_cos_f32(angle_el * RAD_TO_DEG, &s, &c);
    float mod_alpha = mod_d * c - mod_q * s;
    float mod_beta  = mod_d * s + mod_q * c;

    /* SVM → 占空比 */
    SVM(mod_alpha, mod_beta, &d_u, &d_v, &d_w);

    /* 写入 PWM 寄存器 */
    set_pwm_duty(d_u, d_v, d_w);
}
