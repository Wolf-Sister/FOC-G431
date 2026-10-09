/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器实现 — Tustin 离散化、抗积分饱和、输出速率限制
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "utils.h"
#include "foc.h"

/* 全局 PID 实例定义 -------------------------------------------*/
struct PIDController current_loop = {
    .P = IQ_CURRENT_KP_DEFAULT,
    .I = IQ_CURRENT_KI_DEFAULT,
    .limit = CURRENT_VOLTAGE_LIMIT_DEFAULT
};
struct PIDController id_current_loop = {
    .P = ID_CURRENT_KP_DEFAULT,
    .I = ID_CURRENT_KI_DEFAULT,
    .limit = CURRENT_VOLTAGE_LIMIT_DEFAULT
};
struct PIDController speed_loop = {
    .P = SPEED_KP_DEFAULT,
    .I = SPEED_KI_DEFAULT,
    .limit = SPEED_CURRENT_LIMIT_DEFAULT
};

/**
  * @brief  设置电流环 PID 参数
  */
void foc_set_current_pid(float P, float I, float D, float ramp)
{
    current_loop.P           = P;
    current_loop.I           = I;
    current_loop.D           = D;
    current_loop.output_ramp = ramp;
}

/**
  * @brief  设置 Id 电流环 PID 参数
  */
void foc_set_id_current_pid(float P, float I, float D, float ramp)
{
    id_current_loop.P           = P;
    id_current_loop.I           = I;
    id_current_loop.D           = D;
    id_current_loop.output_ramp = ramp;
}

/**
  * @brief  PID 核心更新 — Tustin 梯形积分器 + 速率限制
  * @param  pid   指向 PID 控制器状态的指针
  * @param  error 当前误差（目标值 - 测量值）
  * @return 控制输出，限幅在 +/- pid->limit 范围内
  */
float PIDController_Update(struct PIDController *pid, float error)
{
    /* 计算时间间隔，防止非正常 Ts 值 */
    uint32_t timestamp_now = dwt_get_cycles();
    uint32_t elapsed_cycles = timestamp_now - pid->timestamp_prev_cycles;
    float Ts = dwt_cycles_to_seconds(elapsed_cycles);
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
    pid->sample_time = Ts;

    /* 比例项 */
    float proportional = pid->P * error;

    /* 积分项 — 梯形 (Tustin) 积分                      */
    float integral_next = pid->integral_prev
                          + pid->I * Ts * 0.5f * (error + pid->error_prev);
    /* 软限幅作为深度防御                                  */
    if (integral_next >  pid->limit) integral_next =  pid->limit;
    if (integral_next < -pid->limit) integral_next = -pid->limit;

    /* 微分项 */
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    /* 预限幅求和 — 用于饱和方向判断                      */
    float output_next = proportional + integral_next + derivative;

    /* ── 双向饱和检测（条件积分） ──
     * 如果输出超出限幅且误差仍然朝同一方向推动
     * (error * output_next > 0)，说明系统深度饱和 /
     * 卡死。冻结积分器以防止饱和累积。否则正常更新
     * 积分器，使误差反转时能立即恢复。                    */
    if ((output_next >  pid->limit && error * output_next > 0.0f) ||
        (output_next < -pid->limit && error * output_next > 0.0f))
    {
        /* 深度饱和 + 同向推动 → 冻结积分器               */
    }
    else
    {
        /* 正常调节或正在从饱和恢复                        */
        pid->integral_prev = integral_next;
    }

    /* 将输出钳位到最终限幅值 */
    float output = output_next;
    if (output >  pid->limit) output =  pid->limit;
    if (output < -pid->limit) output = -pid->limit;

    /* 输出速率限制，使响应平滑 */
    if (pid->output_ramp > 0.0f) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }

    /* 保存状态供下一次迭代 */
    pid->output_prev     = output;
    pid->error_prev      = error;
    pid->timestamp_prev_cycles = timestamp_now;

    return output;
}

/** Back-calculate an integrator from downstream vector saturation. */
void PIDController_ApplyTracking(struct PIDController *pid,
                                 float output_correction, float tracking_gain)
{
    if (tracking_gain <= 0.0f || pid->sample_time <= 0.0f) {
        return;
    }

    float integral = pid->integral_prev
                   + tracking_gain * output_correction * pid->sample_time;
    if (integral >  pid->limit) integral =  pid->limit;
    if (integral < -pid->limit) integral = -pid->limit;
    pid->integral_prev = integral;
}


/**
  * @brief  一次性初始化电流环 PI 控制器
  */
void motor_pid_init(float iq_p, float iq_i, float id_p, float id_i)
{
    motor_config.iq_p_gain = iq_p;
    motor_config.iq_i_gain = iq_i;
    motor_config.id_p_gain = id_p;
    motor_config.id_i_gain = id_i;

    foc_set_current_pid(iq_p, iq_i, 0.0f, 0.0f);
    foc_set_id_current_pid(id_p, id_i, 0.0f, 0.0f);
    current_loop.limit = motor_config.current_voltage_limit;

    uint32_t now = dwt_get_cycles();
    current_loop.integral_prev = 0.0f;
    current_loop.output_prev = 0.0f;
    current_loop.error_prev = 0.0f;
    current_loop.timestamp_prev_cycles = now;
    current_loop.sample_time = 0.0f;
    id_current_loop.integral_prev = 0.0f;
    id_current_loop.output_prev = 0.0f;
    id_current_loop.error_prev = 0.0f;
    id_current_loop.timestamp_prev_cycles = now;
    id_current_loop.sample_time = 0.0f;
    id_current_loop.limit = motor_config.current_voltage_limit;
}

/**
  * @brief  在控制循环上下文中重置速度环动态状态
  */
void speed_pid_reset(void)
{
    speed_loop.integral_prev = 0.0f;
    speed_loop.output_prev = 0.0f;
    speed_loop.error_prev = 0.0f;
    speed_loop.timestamp_prev_cycles = dwt_get_cycles();
    speed_loop.sample_time = 0.0f;
}

/**
  * @brief  初始化速度环 PI 控制器增益
  */
void speed_pid_init(float spd_p, float spd_i)
{
    motor_config.spd_p_gain = spd_p;
    motor_config.spd_i_gain = spd_i;

    speed_loop.P           = spd_p;
    speed_loop.I           = spd_i;
    speed_loop.D           = 0.0f;
    speed_loop.output_ramp = 0.0f;
    speed_loop.limit       = motor_config.speed_current_limit;

    speed_pid_reset();
}
/**
  * @brief  Initialize all control-loop gains from pid.h defaults
  */
void control_pid_init(void)
{
    motor_pid_init(IQ_CURRENT_KP_DEFAULT,
                   IQ_CURRENT_KI_DEFAULT,
                   ID_CURRENT_KP_DEFAULT,
                   ID_CURRENT_KI_DEFAULT);
    speed_pid_init(SPEED_KP_DEFAULT, SPEED_KI_DEFAULT);
    motor_config.pos_p_gain = POSITION_KP_DEFAULT;
}
