#include "sample.h"

// 电流环使能标志
volatile uint8_t current_loop_enable = 0;

// 定义全局变量来存储ADC原始值
volatile uint16_t adc_raw_ia = 0;
volatile uint16_t adc_raw_ic = 0;

// 校准采样芯片零点值
void ADC_Calibration(uint16_t cnt) {
    HAL_Delay(10);  // 等待放大器稳定

    int32_t ia_sum = 0;
    int32_t ic_sum = 0;

    for (uint16_t i = 0; i < cnt; i++) 
	  {  
        ia_sum += adc_raw_ia;
        ic_sum += adc_raw_ic;

        HAL_Delay(1);  // 给ADC稳定时间
    }

    motor_control.IphA_offset = ia_sum / cnt;
    motor_control.IphC_offset = ic_sum / cnt;

    printf("ADC Zero Calibration Done!\r\n");
	printf("Mid_A:%d\tMid_C:%d\r\n",motor_control.IphA_offset,motor_control.IphC_offset);
}

// 将采样到的模拟电压值转换为实际电流值
float VoltageToCurrent(uint32_t ADCValue, uint16_t offset_adc) {
    int adcval_diff = (int)ADCValue - (int)offset_adc;
    float amp_voltage = (ADC_REFERENCE_VOLT / ADC_BITS) * adcval_diff;  // 实际上是放大后的电压
    float current = amp_voltage / (OP_GAIN * R_SHUNT);  			    // 除以增益和分流电阻
    return current;
}

/**
  * @brief  注入转换完成回调函数
  * @param  hadc ADC句柄指针
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    // 从注入组读取数值
    adc_raw_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adc_raw_ic = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	  // 获取计算三相电流
    motor_control.IphA = VoltageToCurrent(adc_raw_ia, motor_control.IphA_offset);
    motor_control.IphC = - VoltageToCurrent(adc_raw_ic, motor_control.IphC_offset);
    motor_control.IphB = - motor_control.IphC - motor_control.IphA; // 电路设计C相和A相采样方向相反
    // 底层电流环
    if(current_loop_enable == 1) {
      foc_current_loop();
    }
  }
}