/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID controller implementation — Tustin discretization, anti-windup,
  *          output slew-rate limiting
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "utils.h"
#include "foc.h"

/* PID limits ----------------------------------------------------------------*/
#define MAX_ANGLE_SPEED     100.0f           /* Max angular speed output    */
#define MAX_IQ_CURRENT      LIMIT_CURRENT    /* Max Iq current (A)           */
#define MAX_MODULATION      12 * 0.9f        /* Max modulation voltage       */

/* Global PID instance definitions -------------------------------------------*/
struct PIDController angle_loop   = {.P = 2.0f, .I = 0.0f,  .limit = MAX_ANGLE_SPEED};
struct PIDController vel_loop     = {.P = 2.0f, .I = 20.0f, .limit = MAX_IQ_CURRENT};
struct PIDController current_loop = {.P = 1.0f, .I = 10.0f, .limit = MAX_MODULATION};

/* --------------------------------------------------------------------------*/
/**
  * @brief  Set angle-loop PID parameters
  */
void foc_set_angle_pid(float P, float I, float D, float ramp, float limit)
{
    angle_loop.P           = P;
    angle_loop.I           = I;
    angle_loop.D           = D;
    angle_loop.output_ramp = ramp;
    angle_loop.limit       = limit;
}

/**
  * @brief  Set velocity-loop PID parameters
  */
void foc_set_vel_pid(float P, float I, float D, float ramp, float limit, float alpha)
{
    vel_loop.P           = P;
    vel_loop.I           = I;
    vel_loop.D           = D;
    vel_loop.output_ramp = ramp;
    vel_loop.limit       = limit;
    motor_control.vel_lowpass_alpha = alpha;
}

/**
  * @brief  Set current-loop PID parameters
  */
void foc_set_current_pid(float P, float I, float D, float ramp)
{
    current_loop.P           = P;
    current_loop.I           = I;
    current_loop.D           = D;
    current_loop.output_ramp = ramp;
}

/**
  * @brief  Core PID update — Tustin trapezoidal integrator + rate limiting
  * @param  pid   pointer to PID controller state
  * @param  error current error (setpoint - measurement)
  * @return control output, clamped to +/- pid->limit
  */
float PIDController_Update(struct PIDController *pid, float error)
{
    /* Calculate time delta, protect against unrealistic Ts */
    unsigned long timestamp_now = dwt_get_micros();
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    /* Proportional term */
    float proportional = pid->P * error;

    /* Integral term — trapezoidal (Tustin) integration */
    float integral = pid->integral_prev
                     + pid->I * Ts * 0.5f * (error + pid->error_prev);
    if (integral >  pid->limit) integral =  pid->limit;
    if (integral < -pid->limit) integral = -pid->limit;

    /* Derivative term */
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    /* Sum */
    float output = proportional + integral + derivative;

    /* Output saturation */
    if (output >  pid->limit) output =  pid->limit;
    if (output < -pid->limit) output = -pid->limit;

    /* Slew-rate limiting for smooth response */
    if (pid->output_ramp > 0.0f) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }

    /* Store state for next iteration */
    pid->integral_prev   = integral;
    pid->output_prev     = output;
    pid->error_prev      = error;
    pid->timestamp_prev  = timestamp_now;

    return output;
}

/**
  * @brief  One-shot init of all three PID loops from high-level gains
  */
void motor_pid_init(float tor_p, float tor_i, float vel_p, float vel_i, float pos_p)
{
    motor_config.torque_gain            = tor_p;
    motor_config.torque_integrator_gain = tor_i;
    motor_config.vel_gain               = vel_p;
    motor_config.vel_integrator_gain    = vel_i;
    motor_config.pos_gain               = pos_p;

    foc_set_angle_pid(motor_config.pos_gain, 0.0f, 0.0f, 100000.0f, LIMIT_CURRENT);
    foc_set_vel_pid(motor_config.vel_gain, motor_config.vel_integrator_gain,
                    0.0f, 100000.0f, LIMIT_CURRENT, VEL_ALPHA);
    foc_set_current_pid(motor_config.torque_gain,
                        motor_config.torque_integrator_gain, 0.0f, 0.0f);
}
