#include "pid.h"

#define MAX_ANGLE_SPEED        100.0f          // 最大角度速度限幅
#define MAX_IQ_CURRENT         LIMIT_CURRENT   // 最大Iq电流限幅
#define MAX_MODULATION         12 * 0.9f       // 电流环输出的最大调制电压

struct PIDController angle_loop  = {.P = 2.0f, .I = 0.0f,  .limit = MAX_ANGLE_SPEED};  // 输出Iq(A)
struct PIDController vel_loop    = {.P = 2.0f, .I = 20.0f, .limit = MAX_IQ_CURRENT};   // 输出Iq(A)
struct PIDController current_loop= {.P = 1.0f, .I = 10.0f, .limit = MAX_MODULATION};   // 输出归一化电压(0~1)

/**
 * @brief   设置电机速度环的 PID 参数
 * @param   P       比例增益 
 * @param   I       积分增益
 * @param   D       微分增益 
 * @param   ramp    输出斜率限制 单位/秒
 * @param   limit   输出限幅
 * @param   alpha   速度测量值的低通滤波系数
 */
void foc_set_vel_pid(float P, float I, float D, float ramp, float limit, float alpha)   
{
	vel_loop.P=P;
	vel_loop.I=I;
	vel_loop.D=D;
	vel_loop.output_ramp=ramp;
	vel_loop.limit=limit;
	motor_control.vel_lowpass_alpha = alpha;
}

/**
 * @brief   设置电机角度环的 PID 参数
 * @param   P       比例增益
 * @param   I       积分增益
 * @param   D       微分增益
 * @param   ramp    输出斜率限制
 * @param   limit   输出限幅
 */
void foc_set_angle_pid(float P,float I,float D,float ramp,float limit)   
{
	angle_loop.P=P;
	angle_loop.I=I;
	angle_loop.D=D;
	angle_loop.output_ramp=ramp;
	angle_loop.limit=limit;
}    

/**
 * @brief	设置电机电流环的 PID 参数
 * @param   P       比例增益
 * @param   I       积分增益
 * @param   D       微分增益
 * @param   ramp    输出斜率限制
 */
void foc_set_current_pid(float P,float I,float D,float ramp)  
{
	current_loop.P=P;
	current_loop.I=I;
	current_loop.D=D;
	current_loop.output_ramp=ramp;
}

/**
 * @brief   更新 PID 控制器的状态并计算输出
 * @param   pid     指向 PID 控制器结构体的指针
 * @param   error   当前的输入误差
 * @return  float   PID 控制器的计算输出
 */
float PIDController_Update(struct PIDController* pid, float error) {
    // 获取当前时间戳
    unsigned long timestamp_now = dwt_get_micros(); 

    // 防止系统刚启动或定时器溢出时出现除零错误
    float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // 比例项
    float proportional = pid->P * error;

    // 积分项 使用Tustin（梯形）方法进行离散化 比简单的前向欧拉法更精确
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    if (integral > pid->limit) integral = pid->limit;
    else if (integral < -pid->limit) integral = -pid->limit;

    // 微分项
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // 计算总输出
    float output = proportional + integral + derivative;

    // 输出限幅
    if (output > pid->limit) output = pid->limit;
    else if (output < -pid->limit) output = -pid->limit;

    // 输出斜率限制 防止输出值突变 使系统响应更平滑
    if (pid->output_ramp > 0) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }

    // 更更新状态变量，为下一次迭代做准备
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;

    return output;
}

/**
 * @brief   根据高级别增益参数初始化所有电机 PID 控制器
 * @param   tor_p   转矩环(电流环) P增益
 * @param   tor_i   转矩环(电流环) I增益
 * @param   vel_p   速度环 P增益
 * @param   vel_i   速度环 I增益
 * @param   pos_p   位置环(角度环) P增益
 */
void motor_pid_init(float tor_p, float tor_i, float vel_p, float vel_i, float pos_p)
{
	motor_config.torque_gain = tor_p;
	motor_config.torque_integrator_gain = tor_i;
	motor_config.vel_gain = vel_p;
	motor_config.vel_integrator_gain = vel_i;
	motor_config.pos_gain = pos_p;
	
	foc_set_angle_pid(motor_config.pos_gain, 0, 0, 100000, LIMIT_CURRENT);  // 位置环内环为电流环
    foc_set_vel_pid(motor_config.vel_gain, motor_config.vel_integrator_gain , 0.00, 100000, LIMIT_CURRENT, VEL_ALPHA); 
    foc_set_current_pid(motor_config.torque_gain, motor_config.torque_integrator_gain, 0, 0);
}

