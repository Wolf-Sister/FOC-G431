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
#include "uart_tx.h"
#include "as5047p.h"
#include "pid.h"
#include <math.h>
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
#include "ina219.h"

/* ========================================================================== */
/*  全局变量                                                                   */
/* ========================================================================== */

/* 相电流（双 ADC 采集） */
volatile Phase_Current_t motor_current = {0};

/* 电流环使能 — 校准 + 对齐完成后设置 */
volatile uint8_t current_loop_enable = 0;

/* Set only after all motor-control state has been initialized. */
volatile uint8_t motor_ready = 0;

volatile motor_fault_t motor_fault_reason = MOTOR_FAULT_NONE;
volatile motor_fault_snapshot_t motor_fault_snapshot;
static current_protection_t current_protection;
static volatile uint8_t calibration_reset_pending = 1U;
static volatile uint8_t adc_progress_armed = 0U;
static volatile uint32_t adc_last_cycle = 0U;
static volatile uint32_t calibration_start_ms = 0U;
static volatile uint8_t driver_monitor_enabled = 0U;
static uint32_t alignment_start_ms;

/* 传感器对齐进行中 — TIM 回调不得覆盖 PWM */
volatile uint8_t alignment_in_progress = 0;

/* 编码器掉线锁存 — 置位后电流环不再输出，需复位 MCU 恢复 */
volatile uint8_t encoder_lost_latched = 0;
volatile uint32_t encoder_stale_count = 0;

/* PWM 占空比被限幅标志 — 供电流环做抗饱和回灌 */
volatile uint8_t pwm_duty_clipped = 0;

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
    if (motor_fault_reason != MOTOR_FAULT_NONE) return;
    if (!isfinite(Ud) || !isfinite(Uq) || !isfinite(angle) || period > 65535U ||
        period > htim1.Instance->ARR) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
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
    if (!isfinite(ta) || !isfinite(tb) || !isfinite(tc) ||
        ta < 0.0f || tb < 0.0f || tc < 0.0f ||
        ta > 65535.0f || tb > 65535.0f || tc > 65535.0f) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }

    uint16_t ccr1 = 0, ccr2 = 0, ccr3 = 0;
    switch (sector) {
        case 1: ccr1 = (uint16_t)ta; ccr2 = (uint16_t)tb; ccr3 = (uint16_t)tc; break;
        case 2: ccr1 = (uint16_t)tb; ccr2 = (uint16_t)ta; ccr3 = (uint16_t)tc; break;
        case 3: ccr1 = (uint16_t)tc; ccr2 = (uint16_t)ta; ccr3 = (uint16_t)tb; break;
        case 4: ccr1 = (uint16_t)tc; ccr2 = (uint16_t)tb; ccr3 = (uint16_t)ta; break;
        case 5: ccr1 = (uint16_t)tb; ccr2 = (uint16_t)tc; ccr3 = (uint16_t)ta; break;
        case 6: ccr1 = (uint16_t)ta; ccr2 = (uint16_t)tc; ccr3 = (uint16_t)tb; break;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_fault_reason != MOTOR_FAULT_NONE) {
        __set_PRIMASK(primask);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
    __set_PRIMASK(primask);
}

/* ========================================================================== */
/*  校准 & 角度滤波器（保留）                                                   */
/* ========================================================================== */

void Motor_Current_Calibration(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Motor_HardwareDisable();
    current_loop_enable = 0U;
    motor_ready = 0U;
    driver_monitor_enabled = 0U;
    adc_progress_armed = 0U;
    calibration_start_ms = HAL_GetTick();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    UART2_SendString("[FOC] Calibrating Current Sensor Offset via Interrupt...\r\n");

    motor_current.Offset_A  = 0;
    motor_current.Offset_C  = 0;
    motor_current.Calibrated = 0;
    calibration_reset_pending = 1U;
    __set_PRIMASK(primask);
}

/**
  * @brief  向 UART2 发送一段以 '\0' 结尾的字符串
  * @note   改为写入 TX 环形缓冲，不再直接操作 DMA。
  *         原实现有三个缺陷:
  *           1) while (gState != READY) 无超时，UART 报错即死循环
  *           2) 把调用者的栈缓冲交给异步 DMA，函数返回后缓冲被复用
  *           3) 不可重入，却被主上下文与对齐流程共用
  *         现由 uart_tx 模块统一排队，实际搬运只在主循环 UART_TX_Pump()
  *         中进行；日志生产者必须串行调用，运行期统一由主循环输出。
  */
void UART2_SendString(const char *str)
{
    (void)UART_TX_PutString(str);
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
    if (!isfinite(current_voltage_limit) || !isfinite(speed_current_limit) ||
        !isfinite(position_speed_limit)) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
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
    float reference_sq = motor_control.set_torque * motor_control.set_torque +
                         motor_control.id_target * motor_control.id_target;
    if (reference_sq > speed_current_limit * speed_current_limit) {
        float scale = speed_current_limit / sqrtf(reference_sq);
        motor_control.set_torque *= scale;
        motor_control.id_target *= scale;
    }

    motor_config.pos_speed_limit = position_speed_limit;

    __set_PRIMASK(primask);
}

/* ========================================================================== */
/*  安全停机                                                                    */
/* ========================================================================== */

/**
  * @brief  立即关断功率级并清空全部控制器动态状态
  * @note   可在任意上下文调用（含中断与 Error_Handler）。设计为:
  *           - 幂等: 重复调用无副作用
  *           - 不依赖 HAL 返回值，直接写 TIM1/GPIO 寄存器
  *           - 关断顺序: 驱动器禁能 → MOE关闭 → 清CCR及软件状态
  *
  *         PB13/PB14/PB15 为 DRV8323H 3x PWM 模式的 INLx 绑定脚，必须保持
  *         高电平才能工作，因此本函数不得触碰它们。
  *
  *         拉低 DRV_EN 后驱动器输出级断电；由于未实现自动恢复流程，
  *         故障停机后需复位 MCU 才能重新使能。
  */
void foc_safe_stop(void)
{
    motor_fault_trip(MOTOR_FAULT_STOP);
}

void motor_fault_trip(motor_fault_t reason)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t first_fault = motor_fault_reason == MOTOR_FAULT_NONE;
    if (motor_fault_reason == MOTOR_FAULT_NONE) {
        motor_fault_reason = reason == MOTOR_FAULT_NONE ? MOTOR_FAULT_STOP : reason;
    }
    Motor_HardwareDisable();
    if (first_fault) {
        motor_fault_snapshot.tick_ms = HAL_GetTick();
        motor_fault_snapshot.raw_a = motor_current.Raw_A;
        motor_fault_snapshot.raw_c = motor_current.Raw_C;
        motor_fault_snapshot.offset_a = motor_current.Offset_A;
        motor_fault_snapshot.offset_c = motor_current.Offset_C;
        motor_fault_snapshot.ia = current_protection.ia;
        motor_fault_snapshot.ib = current_protection.ib;
        motor_fault_snapshot.ic = current_protection.ic;
        motor_fault_snapshot.was_aligning = alignment_in_progress;
        motor_fault_snapshot.was_running = current_loop_enable;
    }

    /* 4. 清空全部 PID 动态状态 —— 这是本函数的核心价值。
     *    否则故障自恢复时，积分器里积攒的满幅指令会瞬间加到功率级。 */
    current_loop.integral_prev = 0.0f;
    current_loop.output_prev   = 0.0f;
    current_loop.error_prev    = 0.0f;

    id_current_loop.integral_prev = 0.0f;
    id_current_loop.output_prev   = 0.0f;
    id_current_loop.error_prev    = 0.0f;

    speed_loop.integral_prev = 0.0f;
    speed_loop.output_prev   = 0.0f;
    speed_loop.error_prev    = 0.0f;

    /* 5. 清滤波器、调制量与目标值 */
    motor_control.id_filter_state = 0.0f;
    motor_control.iq_filter_state = 0.0f;
    motor_control.mod_d  = 0.0f;
    motor_control.mod_q  = 0.0f;
    motor_control.du     = 0.0f;
    motor_control.dv     = 0.0f;
    motor_control.dw     = 0.0f;
    motor_control.set_torque = 0.0f;
    motor_control.set_speed  = 0.0f;
    motor_control.id_target = 0.0f;
    motor_control.set_position = motor_control.pos_meas;
    motor_control.id_set = 0.0f;
    motor_control.iq_set = 0.0f;
    motor_control.vel_filter_state = 0.0f;
    motor_control.spd_needs_init = 1U;
    motor_ready = 0U;
    alignment_in_progress = 0U;
    driver_monitor_enabled = 0U;
    current_loop_enable = 0U;

    __set_PRIMASK(primask);
}

bool foc_enable_driver(void)
{
    uint32_t sensor_start = HAL_GetTick();
    uint8_t sensor_ready = 0U;
    do {
        if (motor_fault_reason != MOTOR_FAULT_NONE) return false;
        if (AS5047P_Sensor_Update(&AngleSensor) &&
            (uint32_t)(dwt_get_cycles() - AngleSensor.sample_cycle) <
            (uint32_t)((float)SystemCoreClock * AS5047P_STALE_LIMIT_S)) {
            sensor_ready = 1U;
            break;
        }
    } while ((uint32_t)(HAL_GetTick() - sensor_start) < FOC_ALIGN_SAMPLE_TIMEOUT_MS);
    if (!sensor_ready) {
        motor_fault_trip(MOTOR_FAULT_ENCODER);
        return false;
    }
    uint32_t primask = __get_PRIMASK();
    uint32_t start, stable_since;
    __disable_irq();
    if (motor_fault_reason != MOTOR_FAULT_NONE || !motor_current.Calibrated ||
        !AS5047P_DiagnosticsHealthy()) {
        __set_PRIMASK(primask);
        return false;
    }
    if (!ina219_bus_valid || !isfinite(motor_config.voltage_supply) ||
        motor_config.voltage_supply < FOC_BUS_MIN_VALID_V ||
        motor_config.voltage_supply > FOC_BUS_MAX_VALID_V ||
        (uint32_t)(HAL_GetTick() - ina219_bus_sample_ms) >= FOC_BUS_MAX_AGE_MS) {
        __set_PRIMASK(primask);
        motor_fault_trip(MOTOR_FAULT_BUS);
        return false;
    }
    TIM1->BDTR |= TIM_BDTR_MOE;
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
    __set_PRIMASK(primask);
    start = HAL_GetTick();
    stable_since = start;
    do {
        if (motor_fault_reason != MOTOR_FAULT_NONE) return false;
        if (HAL_GPIO_ReadPin(NFAULT_GPIO_Port, NFAULT_Pin) == GPIO_PIN_RESET) {
            stable_since = HAL_GetTick();
        } else if ((uint32_t)(HAL_GetTick() - stable_since) >= 5U) {
            primask = __get_PRIMASK();
            __disable_irq();
            if (motor_fault_reason == MOTOR_FAULT_NONE &&
                HAL_GPIO_ReadPin(NFAULT_GPIO_Port, NFAULT_Pin) == GPIO_PIN_SET) {
                driver_monitor_enabled = 1U;
                __set_PRIMASK(primask);
                return true;
            }
            __set_PRIMASK(primask);
        }
        HAL_Delay(1U);
    } while ((uint32_t)(HAL_GetTick() - start) < FOC_DRIVER_READY_TIMEOUT_MS);
    motor_fault_trip(MOTOR_FAULT_DRIVER);
    return false;
}

bool foc_enable_control(void)
{
    uint32_t primask = __get_PRIMASK();
    bool enabled = false;
    encoder_cache_t encoder = {0};
    __disable_irq();
    if (motor_fault_reason == MOTOR_FAULT_NONE && motor_current.Calibrated &&
        driver_monitor_enabled && !alignment_in_progress &&
        AS5047P_DiagnosticsHealthy() && AS5047P_EncoderCache_Read(&encoder) &&
        encoder.data_valid && !encoder.stale) {
        current_loop_enable = 1U;
        motor_ready = 1U;
        enabled = true;
    }
    __set_PRIMASK(primask);
    return enabled;
}

void foc_process_current_sample(uint16_t raw_a, uint16_t raw_c)
{
    current_sample_result_t result;
    motor_current.Raw_A = raw_a;
    motor_current.Raw_C = raw_c;
    adc_last_cycle = dwt_get_cycles();
    adc_progress_armed = 1U;
    if (motor_fault_reason != MOTOR_FAULT_NONE) return;
    if (motor_current.Calibrated && !AS5047P_DiagnosticsHealthy()) {
        motor_fault_trip(MOTOR_FAULT_ENCODER);
        return;
    }
    if (calibration_reset_pending) {
        /* Statistics are reset at the ADC boundary, never during a frame. */
        current_protection_reset(&current_protection);
        calibration_reset_pending = 0U;
    }
    if (driver_monitor_enabled &&
        HAL_GPIO_ReadPin(NFAULT_GPIO_Port, NFAULT_Pin) == GPIO_PIN_RESET) {
        motor_fault_trip(MOTOR_FAULT_DRIVER);
        return;
    }
    if (driver_monitor_enabled &&
        (uint32_t)(HAL_GetTick() - ina219_bus_sample_ms) >= FOC_BUS_MAX_AGE_MS) {
        motor_fault_trip(MOTOR_FAULT_BUS);
        return;
    }
    result = current_protection_sample(&current_protection, raw_a, raw_c);
    if (result == CURRENT_SAMPLE_CALIBRATING) return;
    if (result == CURRENT_SAMPLE_CALIBRATED) {
        motor_current.Offset_A = current_protection.offset_a;
        motor_current.Offset_C = current_protection.offset_c;
        motor_current.Calibrated = 1U;
        return;
    }
    if (result != CURRENT_SAMPLE_VALID) {
        motor_fault_t reason = MOTOR_FAULT_NONFINITE;
        if (result == CURRENT_SAMPLE_ADC_RANGE) reason = MOTOR_FAULT_ADC_RANGE;
        if (result == CURRENT_SAMPLE_CAL_INVALID) reason = MOTOR_FAULT_CALIBRATION;
        if (result == CURRENT_SAMPLE_OVERCURRENT) reason = MOTOR_FAULT_OVERCURRENT;
        motor_fault_trip(reason);
        return;
    }
    motor_current.I_A = current_protection.ia;
    motor_current.I_B = current_protection.ib;
    motor_current.I_C = current_protection.ic;
    foc_sync_phase_currents();
    if (current_loop_enable) foc_current_loop();
}

void foc_check_sampling_progress(void)
{
    if (motor_fault_reason == MOTOR_FAULT_NONE && adc_progress_armed &&
        (uint32_t)(dwt_get_cycles() - adc_last_cycle) >=
        (SystemCoreClock / 1000U) * CURRENT_ADC_TIMEOUT_MS) {
        motor_fault_trip(MOTOR_FAULT_ADC_TIMEOUT);
    }
}

void foc_check_calibration_timeout(void)
{
    if (motor_fault_reason == MOTOR_FAULT_NONE && !motor_current.Calibrated &&
        (uint32_t)(HAL_GetTick() - calibration_start_ms) >= CURRENT_CAL_TIMEOUT_MS) {
        motor_fault_trip(MOTOR_FAULT_CALIBRATION);
    }
}

/* ========================================================================== */
/*  电气角度辅助函数 (来自 TinyFoc utils.c)                                    */
/* ========================================================================== */

/**
  * @brief  在传感器对齐过程中计算零电角偏移量
  */
static HAL_StatusTypeDef foc_calculate_zero(float *zero)
{
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    uint8_t valid_samples = 0U;
    uint32_t start = HAL_GetTick();

    while (valid_samples < 10U) {
        if (motor_fault_reason != MOTOR_FAULT_NONE) return HAL_ERROR;
        if ((uint32_t)(HAL_GetTick() - start) >= FOC_ALIGN_SAMPLE_TIMEOUT_MS ||
            (alignment_in_progress && (uint32_t)(HAL_GetTick() - alignment_start_ms) >=
             FOC_ALIGN_TOTAL_TIMEOUT_MS)) {
            motor_fault_trip(MOTOR_FAULT_ALIGN_TIMEOUT);
            return HAL_TIMEOUT;
        }
        if (AS5047P_Sensor_Update(&AngleSensor) &&
            (uint32_t)(dwt_get_cycles() - AngleSensor.sample_cycle) <
            (uint32_t)((float)SystemCoreClock * AS5047P_STALE_LIMIT_S)) {
            float angle = AS5047P_GetAngle(&AngleSensor);
            if (!isfinite(angle)) {
                motor_fault_trip(MOTOR_FAULT_NONFINITE);
                return HAL_ERROR;
            }
            float s;
            float c;
            arm_sin_cos_f32(angle * RAD_TO_DEG, &s, &c);
            sum_sin += s;
            sum_cos += c;
            valid_samples++;
            /* Spread accepted samples in time; poll discarded/old pipeline
             * responses without adding another 1 ms of sample latency. */
            HAL_Delay(1U);
        }
    }

    if (sum_sin * sum_sin + sum_cos * sum_cos <
        100.0f * FOC_ALIGN_MIN_RESULTANT * FOC_ALIGN_MIN_RESULTANT) {
        motor_fault_trip(MOTOR_FAULT_ALIGN_UNSTABLE);
        return HAL_ERROR;
    }

    float mech_angle = atan2f(sum_sin, sum_cos);
    if (mech_angle < 0.0f) mech_angle += _2PI;

    float raw_elec_angle = (float)(motor_config.dir * motor_config.pairs) * mech_angle;
    *zero = _normalizeAngle(raw_elec_angle);
    return HAL_OK;
}

float _calculate_zero_electric_angle(void)
{
    float zero;
    if (foc_calculate_zero(&zero) != HAL_OK) return NAN;
    return zero;
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

/* Wait for real 1 ms intervals. HAL_Delay(1) adds a SysTick interval, which
 * accumulates across ramps and can exhaust the total alignment budget. */
static HAL_StatusTypeDef foc_align_wait(uint32_t delay_ms,
                                        uint32_t *overcurrent_ms)
{
    uint32_t elapsed_ms;
    const uint32_t cycles_per_ms = SystemCoreClock / 1000U;

    for (elapsed_ms = 0U; elapsed_ms < delay_ms; elapsed_ms++) {
        uint32_t start_cycle = dwt_get_cycles();
        do {
            if (motor_fault_reason != MOTOR_FAULT_NONE) return HAL_ERROR;
            /* SysTick remains a separate deadline if DWT stops progressing. */
            if ((uint32_t)(HAL_GetTick() - alignment_start_ms) >= FOC_ALIGN_TOTAL_TIMEOUT_MS) {
                motor_fault_trip(MOTOR_FAULT_ALIGN_TIMEOUT);
                return HAL_TIMEOUT;
            }
        } while ((uint32_t)(dwt_get_cycles() - start_cycle) < cycles_per_ms);
        if (motor_fault_reason != MOTOR_FAULT_NONE) return HAL_ERROR;

        if (foc_align_max_phase_current() > FOC_ALIGN_CURRENT_LIMIT_A) {
            (*overcurrent_ms)++;
            if (*overcurrent_ms >= FOC_ALIGN_OVERCURRENT_MS) {
                motor_fault_trip(MOTOR_FAULT_OVERCURRENT);
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
    if (motor_fault_reason != MOTOR_FAULT_NONE || !driver_monitor_enabled) return HAL_ERROR;
    alignment_start_ms = HAL_GetTick();

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
    float zero;
    if (foc_calculate_zero(&zero) != HAL_OK) goto alignment_failed;
    motor_control.zero_elec_angle = zero;

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
    if (motor_fault_reason != MOTOR_FAULT_NONE) goto alignment_failed;
    alignment_in_progress = 0;

    (void)snprintf(log_buf, sizeof(log_buf),
                   "[FOC] Zero elec angle = %.3f rad\r\n",
                   motor_control.zero_elec_angle);
    UART2_SendString(log_buf);
    UART2_SendString("[FOC] Encoder Calibration Done!\r\n");
    return HAL_OK;

alignment_failed:
    motor_fault_trip(MOTOR_FAULT_ALIGN_TIMEOUT);
    if (encoder_timer_stopped != 0U) {
        AS5047P_DMA_StartRequest();
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim2);
    }
    alignment_in_progress = 0;
    UART2_SendString("[FOC] Alignment aborted: fault latched\r\n");
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
static void foc_duty_to_dq(float d_u, float d_v, float d_w,
                           float s, float c, float *Vd_act, float *Vq_act);
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
    if (motor_fault_reason != MOTOR_FAULT_NONE || !current_loop_enable) return;
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
    if (!isfinite(planned_speed) || !isfinite(curr_pos)) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_fault_reason == MOTOR_FAULT_NONE) {
        motor_control.set_speed = planned_speed;
        motor_control.pos_meas = curr_pos;
    }
    __set_PRIMASK(primask);
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
    /* 编码器掉线已锁存: 控制环彻底停止，不再产生任何输出 */
    if (motor_fault_reason != MOTOR_FAULT_NONE || encoder_lost_latched != 0U) {
        return;
    }

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

    /* ── 1. 按 DMA 实际采样时刻，将编码器角度预测到当前电流帧 ──
     *     encoder.stale 由 AS5047P_EncoderCache_Read() 在读取时刻判定。
     *     两级策略:
     *       1 级 (瞬时陈旧)  : 停止外推，冻结在最后已知角度，并折减电流限幅
     *       2 级 (持续掉线)  : 锁存故障并关断功率级，需复位 MCU 恢复
     *     分级的意义在于: AS5047P 偶发单帧校验错是正常的，逐帧停机不实用;
     *     而连续 20 帧 (1 ms @20kHz) 无有效数据已超出任何可恢复的暂态范围。  */
    float angle_mech = encoder.angle_raw;
    float angle_age_s = dwt_cycles_to_seconds(encoder.age_cycles);
    float current_limit_scale = 1.0f;

    if (encoder.stale != 0U) {
        encoder_stale_count++;

        if (encoder_stale_count >= FOC_ENCODER_LOST_FRAMES) {
            encoder_lost_latched = 1U;
            motor_fault_trip(MOTOR_FAULT_ENCODER);
            return;
        }

        /* 1 级: 不外推 (用最后已知角度)，并折减电流限幅 */
        angle_age_s = 0.0f;
        current_limit_scale = FOC_ENCODER_DEGRADE_SCALE;
    } else {
        encoder_stale_count = 0U;
        if (angle_age_s > FOC_ENCODER_PREDICTION_MAX_S) {
            angle_age_s = FOC_ENCODER_PREDICTION_MAX_S;
        }
    }
    angle_mech += encoder.velocity_rad_s * angle_age_s;

    float angle_el = (float)(motor_config.dir * motor_config.pairs)
                     * angle_mech
                     - motor_control.zero_elec_angle;
    if (!isfinite(angle_el) || !isfinite(encoder.velocity_rad_s) ||
        !isfinite(motor_control.set_torque) || !isfinite(motor_control.id_target) ||
        !isfinite(motor_config.speed_current_limit) || motor_config.speed_current_limit <= 0.0f) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
    angle_el = _normalizeAngle(angle_el);
#if FOC_FEEDFORWARD_ENABLE
    float elec_vel = (float)(motor_config.dir * motor_config.pairs)
                     * encoder.velocity_rad_s;
#endif

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
    /* 编码器陈旧时折减限幅 (current_limit_scale = FOC_ENCODER_DEGRADE_SCALE)，
     * 用降低转矩的方式换取角度不可信期间的损伤风险 */
    float current_limit = motor_config.speed_current_limit * current_limit_scale;
    if (current_limit > CURRENT_REF_HARD_MAX_A * current_limit_scale) {
        current_limit = CURRENT_REF_HARD_MAX_A * current_limit_scale;
    }
    /* Bound each axis before squaring, including internal references; the
     * final vector clamp below retains the coupled dq limit. */
    iq_target = _constrain(iq_target, -current_limit, current_limit);
    id_target = _constrain(id_target, -current_limit, current_limit);
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

    /* ── 7. PI 控制器 ── */
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
    pwm_duty_clipped = 0U;
    foc_forward_cordic(Vd, Vq, s, c);

    /* ── 12. 占空比限幅抗饱和 ──
     * set_pwm_duty() 的钳位发生在 dq 电压钳位之后，原先没有任何反馈路径，
     * 导致高调制区积分器持续累积。此处把"实际施加"与"请求值"的差回灌给
     * 积分器。duty → Vabc → αβ → dq 与正向链严格互逆 (中心对齐 PWM 下
     * avg(Vphase) = duty × Vbus)，所以回灌量就是被裁剪掉的那部分电压。   */
    if (pwm_duty_clipped != 0U) {
        float Vd_act;
        float Vq_act;
        foc_duty_to_dq(motor_control.du, motor_control.dv, motor_control.dw,
                       s, c, &Vd_act, &Vq_act);

        PIDController_ApplyTracking(&id_current_loop, Vd_act - Vd,
                                    CURRENT_VECTOR_AW_GAIN_DEFAULT);
        PIDController_ApplyTracking(&current_loop, Vq_act - Vq,
                                    CURRENT_VECTOR_AW_GAIN_DEFAULT);
    }
}

/* ========================================================================== */
/*  CORDIC 加速前向通道 — 预计算 sin/cos → SVM → PWM                          */
/*  在闭环热路径上替代 foc_forward()                                           */
/* ========================================================================== */
static void foc_forward_cordic(float d, float q, float s_ff, float c_ff)
{
    if (motor_fault_reason != MOTOR_FAULT_NONE) return;
    if (!isfinite(d) || !isfinite(q) || !isfinite(s_ff) || !isfinite(c_ff) ||
        !isfinite(motor_config.voltage_supply) || motor_config.voltage_supply <= 0.0f) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
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
  * @brief  将实际写入的 PWM 占空比严格反算回 dq 电压
  * @param  d_u,d_v,d_w  已限幅并实际施加的三相占空比
  * @param  s,c          本电流帧的 sin/cos（与正向链同源，保证严格互逆）
  * @param  Vd_act,Vq_act 输出: 实际施加的 dq 电压
  * @note   中心对齐 PWM 下 avg(V_phase) = duty × Vbus 精确成立，因此
  *         duty → Vabc → αβ → dq 与正向链 dq → αβ → duty 严格互逆。
  *         反 Clarke 用幅值不变约定，与 foc_current_loop() 中的
  *         I_alpha = -(IphB+IphC) / I_beta = (IphB-IphC)/√3 完全对称。
  */
static void foc_duty_to_dq(float d_u, float d_v, float d_w,
                           float s, float c,
                           float *Vd_act, float *Vq_act)
{
    const float vbus = motor_config.voltage_supply;

    /* 1. 占空比 → 相电压 */
    float V_a = d_u * vbus;
    float V_b = d_v * vbus;
    float V_c = d_w * vbus;

    /* 2. 反 Clarke → αβ */
    /* The existing SVM maps positive modulation to decreasing duty. */
    float alpha = -(2.0f * V_a - V_b - V_c) / 3.0f;
    float beta  = -(V_b - V_c) * _1_SQRT3;

    /* 3. 反 Park → dq
     *      [Vd]   [ c  s] [alpha]
     *      [Vq] = [-s  c] [beta ]                                        */
    *Vd_act =  alpha * c + beta * s;
    *Vq_act = -alpha * s + beta * c;
}

/**
  * @brief  将 PWM 占空比限幅后写入 TIM1 CCR 寄存器
  * @note   上限用 PWM_DUTY_MAX (0.98) 而非原先的 0.9。
  *         DRV8323H 集成电荷泵并支持 100% 占空比，0.9 会静默丢掉 SVM
  *         线性区末端约 1.9% 的电压范围；裁剪发生时置位 pwm_duty_clipped，
  *         由 foc_current_loop() 做抗饱和回灌。
  */
static void set_pwm_duty(float d_u, float d_v, float d_w)
{
    if (motor_fault_reason != MOTOR_FAULT_NONE) return;
    if (!isfinite(d_u) || !isfinite(d_v) || !isfinite(d_w)) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
    float c_u = _constrain(d_u, PWM_DUTY_MIN, PWM_DUTY_MAX);
    float c_v = _constrain(d_v, PWM_DUTY_MIN, PWM_DUTY_MAX);
    float c_w = _constrain(d_w, PWM_DUTY_MIN, PWM_DUTY_MAX);

    if (c_u != d_u || c_v != d_v || c_w != d_w) {
        pwm_duty_clipped = 1U;
    }

    motor_control.du = c_u;
    motor_control.dv = c_v;
    motor_control.dw = c_w;

    /* 显式 +0.5f 舍入: 原实现依赖 float→uint32_t 的隐式截断 (向零取整)，
     * 会系统性偏低最多 1 个计数，且属于被关闭的告警所掩盖的类型问题。   */
    const uint32_t arr = htim1.Instance->ARR;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_fault_reason != MOTOR_FAULT_NONE) {
        __set_PRIMASK(primask);
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          (uint32_t)(c_u * (float)arr + 0.5f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          (uint32_t)(c_v * (float)arr + 0.5f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          (uint32_t)(c_w * (float)arr + 0.5f));
    __set_PRIMASK(primask);
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
    if (motor_fault_reason != MOTOR_FAULT_NONE) return;
    if (!isfinite(d) || !isfinite(q) || !isfinite(angle_el) ||
        !isfinite(motor_config.voltage_supply) || motor_config.voltage_supply <= 0.0f) {
        motor_fault_trip(MOTOR_FAULT_NONFINITE);
        return;
    }
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
