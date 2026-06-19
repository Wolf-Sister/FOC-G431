#ifndef __PID__H
#define __PID__H

#include "stm32f4xx_hal.h"
#include "utils.h"
#include "motor.h"
#include "main.h"
#include "foc.h"


extern struct PIDController vel_loop;
extern struct PIDController angle_loop;
extern struct PIDController current_loop;

struct PIDController
{
	float P; //!< 比例增益(P环增益)
	float I; //!< 积分增益（I环增益）
	float D; //!< 微分增益（D环增益）
	float output_ramp; 
	float limit; 
	float ramp; 

	float error_prev; //!< 最后的跟踪误差值
	float output_prev;  //!< 最后一个 pid 输出值
	float integral_prev; //!< 最后一个积分分量值
	unsigned long timestamp_prev; //!< 上次执行时间戳
};

void foc_set_vel_pid(float P, float I, float D, float ramp, float limit, float alpha) ;
void foc_set_angle_pid(float P,float I,float D,float ramp,float limit);
void foc_set_current_pid(float P,float I,float D,float ramp);

float PIDController_Update(struct PIDController* pid, float error);
void motor_pid_init(float tor_p, float tor_i, float vel_p, float vel_i, float pos_p);
#endif