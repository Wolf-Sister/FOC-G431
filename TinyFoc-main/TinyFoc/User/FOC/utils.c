#include "utils.h"

static uint32_t cpu_freq_mhz = 0;

// 初始化DWT
void DWT_Init(void) {
    // 使能 DWT 的 TRCENA 寄存器位
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 使能 CYCCNT 计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 重置计数器
    DWT->CYCCNT = 0;

    // 获取 CPU 频率 (MHz)
    cpu_freq_mhz = SystemCoreClock / 1000000;
}

// 获取系统运行的微秒数
unsigned long dwt_get_micros(void) {
	
    return DWT->CYCCNT / cpu_freq_mhz;
}

// 低通滤波器
float lowPassFilter(float new_angle, float alpha) {
	
    static float filtered_angle = 0.0f; // 初始化为0
    filtered_angle = alpha * new_angle + (1 - alpha) * filtered_angle;
    return filtered_angle;
}

// 角度归一化
float _normalizeAngle(float angle) {
	
	float a = fmodf(angle, 2.0f * PI);
	return (a < 0) ? a + 2.0f * PI : a;
}

// 专门用于在校准时计算零点偏移
float _calculate_zero_electric_angle() {
    // 连续读取几次传感器，取平均值，提高精度
    float sum_angle = 0;
    for (int i=0; i < 10; i++) {
        AS5600_Update(&AngleSensor);
        sum_angle += AS5600_GetAngle(&AngleSensor);
        HAL_Delay(1);
    }
    float mech_angle = sum_angle / 10.0f;

    float raw_elec_angle = (float)(motor_config.dir * motor_config.pairs) * mech_angle;
    return _normalizeAngle(raw_elec_angle);
}

// 获取电角度
float _electricalAngle() {
	float elec_angle = (float)(motor_config.dir * motor_config.pairs) * AS5600_GetAngle(&AngleSensor) - motor_control.zero_elec_angle;
    return  _normalizeAngle(elec_angle);
}

// 获取电角速度
float _electricalVelocity() {
    float elec_velocity = (float)(motor_config.dir * motor_config.pairs) * AS5600_GetVelocity(&AngleSensor);
    return elec_velocity;
}



