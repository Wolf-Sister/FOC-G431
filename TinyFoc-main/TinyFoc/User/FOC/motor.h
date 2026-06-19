#ifndef __MOTOR__H
#define __MOTOR__H

#include "stm32f4xx_hal.h"
#include "fast_trig.h"
#include <stdbool.h>
#include "as5600.h"
#include <stdio.h>
#include "utils.h"
#include "pid.h"


#define MOTOR_VBUS 12		// 电机供电电压
#define LIMIT_CURRENT 5.0f	// 电流限幅

#define VEL_ALPHA 0.05		// 速度低通滤波系数
#define KV_FF 0.002f 	   	// 速度前馈系数，根据实验调整
#define VEL_DEADBAND 1.0f  	// 速度死区阈值

extern struct motor_config_t motor_config;
extern struct motor_control_t motor_control;

struct motor_config_t {
	float voltage_supply; //最大电压
	int dir; 			  //电机方向, 根据电机实际接线调整
	int pairs; 			  //电机极对数（极数/2）
	float pos_gain;
	float vel_gain;
	float vel_integrator_gain;
	float torque_gain;
	float torque_integrator_gain;
};
	

struct motor_control_t {
	float IphA; //电机A相电流
	float IphB; //电机B相电流
	float IphC; //电机C相电流
	uint32_t IphA_offset;
	uint32_t IphB_offset;
	uint32_t IphC_offset;
	float set_pos;   //设置电机位置
	float set_vel;	 //设置电机速度
	float set_torque;//设置电机力矩（电流）
	float Iq_target;
	float pos_target;
	float vel_target;
	uint8_t mode;          //设置电机模式（速度、位置、电流）
	float zero_elec_angle; //电机零电角度
	bool pre_calibrated;   //是否检验过电机
	bool encoder_updated;
	int32_t pos_abs;
	float iq_set;
	float iq_meas;
	float pos_init;
	float du;
	float dv;
	float dw;
	uint32_t latest_ib_raw;
	uint32_t latest_ic_raw;
	float vel_lowpass_alpha;
	float mod_q;
};

/*motor mode*/
typedef enum {
	MOTOR_POSITION, //Position closed loop
	MOTOR_SPEED,	//Speed closed loop
	MOTOR_TORQUE,	//Cureent closed loop
} Motor_Mode_e;	

/* 初始化电机参数 */
void motor_control_parm_init();
/* 设置电机模式 */
void set_motor_mode(Motor_Mode_e mode);
/* 电流变换 */
float cal_Iq_Id(float cur_a, float cur_b, float angle_el);
/* 编码器校准 */
void foc_alignSensor(float q_voltage);
/* 速度环 */
void foc_velocity_loop();
/* 位置环 */
void foc_position_loop();
/* 电流环 */
void foc_current_loop();

#endif

