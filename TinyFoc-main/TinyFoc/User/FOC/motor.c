#include "motor.h"

/*
ToDo: 实现无感磁链观测, 弥补编码器读取频率低于电流环频率的问题
*/

const float current_meas_period = (1 / 5000);	// 电流环频率

struct motor_config_t motor_config = 
{
	.voltage_supply = MOTOR_VBUS, 
	.dir = 1, 
	.pairs = 7,
	.pos_gain = 0,
	.vel_gain = 0,
	.vel_integrator_gain = 0,
	.torque_gain = 0,
	.torque_integrator_gain = 0,
};

struct motor_control_t motor_control = 
{
	.IphA = 0.0,
	.IphB = 0.0,
	.IphC = 0.0, 
	.IphA_offset = 0.0, 
	.IphB_offset = 0.0, 
	.IphC_offset = 0.0, 
	.set_pos = 0, 
	.set_vel = 0, 
	.set_torque = 0, 
	.mode = MOTOR_TORQUE,
	.zero_elec_angle = 0,
	.pre_calibrated = false,
	.encoder_updated = false,
	.pos_abs = 0.0,
	.iq_set = 0.0,
	.iq_meas = 0.0,
	.Iq_target = 0.0,
	.pos_init = 0.0,
	.du = 0.0,
	.dv = 0.0,
	.dw = 0.0,
	.latest_ib_raw = 0,
	.latest_ic_raw = 0,
	.vel_lowpass_alpha = 0.0,
	.mod_q = 0.0,
};

// 电机运行参数初始化
void motor_control_parm_init(void) {
	motor_control.iq_set = 0;
	motor_control.Iq_target = 0;
	motor_control.iq_meas = 0;
	motor_control.set_pos = AS5600_GetAngle(&AngleSensor);
	motor_control.set_vel = 0;
	motor_control.pos_target = 0.0;
	motor_control.vel_target = 0.0;
}

void set_motor_mode(Motor_Mode_e mode) {
	motor_control.mode = mode;
}

float cal_Iq_Id(float cur_b, float cur_c, float angle_el)
{
	// Clarke transform
    float I_alpha = -(cur_b + cur_c);
    float I_beta = _1_SQRT3 * (cur_b - cur_c);
	
	// Park transform
	float I_q = I_beta * fast_cos_f32(angle_el) - I_alpha * fast_sin_f32(angle_el); 
	return I_q;
}

void foc_alignSensor(float q_voltage) {
    // 注入静止电压，让电机稳定
    foc_forward(0, q_voltage, _3PI_2);
    HAL_Delay(1000);

    // 读取编码器多次稳定值 电流环出现问题，和电角度检验关系很大
    AS5600_Update(&AngleSensor); HAL_Delay(10);
	AS5600_Update(&AngleSensor); HAL_Delay(10);
	AS5600_Update(&AngleSensor); HAL_Delay(100);
	// 加大延时时间 保证转子完全静止
	// 获取零电角度采用均值滤波减小高斯噪声影响
	motor_control.zero_elec_angle = _calculate_zero_electric_angle();
	
	foc_forward(0, 0, _3PI_2);
	printf("Set zero_elec_angle = %.3f rd\r\n", motor_control.zero_elec_angle);
	printf("Encoder Calibration Done!\r\n");
}

void foc_position_loop() {
	float pos_error = motor_control.set_pos - AS5600_GetAccumulateAngle(&AngleSensor);
	
	// 死区判断（防止轻微误差导致不断调节）
    if (fabsf(pos_error) < 0.01f) {
		motor_control.Iq_target = 0.0f;
    } else {
		motor_control.Iq_target = PIDController_Update(&angle_loop, pos_error);
    }
}

void foc_velocity_loop() {	
	float vel = AS5600_GetVelocity(&AngleSensor);
	
	if(motor_control.mode == MOTOR_SPEED)
	{
		float error = motor_control.set_vel - vel;
		// 死区
		if (fabsf(motor_control.set_vel) < VEL_DEADBAND && fabsf(error) < VEL_DEADBAND) {
			motor_control.Iq_target = 0.0f;
			vel_loop.integral_prev = 0.0f; // 清除积分项，防止积分累积
			return;  // 直接退出控制环
		}
		
		motor_control.Iq_target = PIDController_Update(&vel_loop, error);
		float ff_out = KV_FF * motor_control.set_vel;	// 前馈输出
		motor_control.Iq_target += ff_out;
	}
}

void foc_current_loop() {
    float error;

    // 对电流值进行低通滤波, 如果值太小会使电流值实时性大幅降低导致电机震动
    motor_control.iq_meas = lowPassFilter(
        cal_Iq_Id(motor_control.IphB, motor_control.IphC, _electricalAngle()), 
        0.1f	// 根据电机实际运行情况调整
    );

    if (motor_control.mode == MOTOR_SPEED || motor_control.mode == MOTOR_POSITION)
	{
        error = motor_control.Iq_target - motor_control.iq_meas;
	}	
    if (motor_control.mode == MOTOR_TORQUE) {
        error = motor_control.set_torque - motor_control.iq_meas;
    }

    // 误差死区,防止过度调节
    if (fabsf(error) < 0.02f) {
        error = 0.0f;
    }

    // PID 计算
    motor_control.iq_set = PIDController_Update(&current_loop, error);

    // 电流死区
    if (fabsf(motor_control.iq_set) < 0.02f) motor_control.iq_set = 0.0f;

	// 限幅
    motor_control.iq_set = _constrain(motor_control.iq_set, -LIMIT_CURRENT, LIMIT_CURRENT);

    // 前馈角延时补偿
    float t_delay = 1.6f * current_meas_period;	// 根据电机运行情况调节前馈系数
    float elec_angle = _electricalAngle();
    float elec_vel = _electricalVelocity();
    float angle_ff = elec_angle + elec_vel * t_delay;

    // SVPWM 输出
    foc_forward(0, motor_control.iq_set, angle_ff);
}

/* 1KHz定时中断 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {
		AS5600_Update(&AngleSensor);	// 更新编码器读数

		if (motor_control.mode == MOTOR_POSITION) {
			foc_position_loop();		// 位置环
		}	
		else if (motor_control.mode == MOTOR_SPEED) {
			foc_velocity_loop();		// 速度环
		}
	}
}