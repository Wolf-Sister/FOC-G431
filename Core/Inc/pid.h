/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID controller with Tustin integrator, output ramp, and anti-windup
  ******************************************************************************
  */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* PID controller state ------------------------------------------------------*/
struct PIDController {
    float P;                  /* Proportional gain                          */
    float I;                  /* Integral gain                              */
    float D;                  /* Derivative gain                            */
    float output_ramp;        /* Max output slew rate (units/sec), 0=off    */
    float limit;              /* Output saturation limit (+/-)              */

    float error_prev;         /* Previous error for derivative + Tustin     */
    float output_prev;        /* Previous output for ramp limiting          */
    float integral_prev;      /* Previous integrator state                  */
    unsigned long timestamp_prev; /* Last update timestamp (us) from DWT    */
};

/* Global PID instances -----------------------------------------------------*/
extern struct PIDController current_loop;     /* Iq current loop              */
extern struct PIDController id_current_loop;  /* Id current loop              */
extern struct PIDController speed_loop;       /* Speed outer loop             */

/* API ----------------------------------------------------------------------*/
float PIDController_Update(struct PIDController *pid, float error);

void  motor_pid_init(float iq_p, float iq_i, float id_p, float id_i);
void  speed_pid_init(float spd_p, float spd_i);
void  foc_set_current_pid(float P, float I, float D, float ramp);
void  foc_set_id_current_pid(float P, float I, float D, float ramp);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
