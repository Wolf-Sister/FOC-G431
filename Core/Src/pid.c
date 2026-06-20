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
#define MAX_MODULATION      12 * 0.9f        /* Max modulation voltage       */

/* Global PID instance definitions -------------------------------------------*/
struct PIDController current_loop    = {.P = 1.0f, .I = 10.0f, .limit = MAX_MODULATION};
struct PIDController id_current_loop = {.P = 1.0f, .I = 10.0f, .limit = MAX_MODULATION};

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
  * @brief  Set Id current-loop PID parameters
  */
void foc_set_id_current_pid(float P, float I, float D, float ramp)
{
    id_current_loop.P           = P;
    id_current_loop.I           = I;
    id_current_loop.D           = D;
    id_current_loop.output_ramp = ramp;
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
  * @brief  One-shot init of current-loop PI controllers
  */
void motor_pid_init(float iq_p, float iq_i, float id_p, float id_i)
{
    motor_config.iq_p_gain = iq_p;
    motor_config.iq_i_gain = iq_i;
    motor_config.id_p_gain = id_p;
    motor_config.id_i_gain = id_i;

    foc_set_current_pid(iq_p, iq_i, 0.0f, 0.0f);
    foc_set_id_current_pid(id_p, id_i, 0.0f, 0.0f);
}
