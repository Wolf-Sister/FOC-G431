#ifndef __SAMPLE__H
#define __SAMPLE__H

#include "motor.h"
#include "stdio.h"
#include "main.h"
#include "adc.h"

// 采样电路参数
#define R_SHUNT 0.01f              // 电流采样电阻
#define OP_GAIN 100.0f             // 运放放大倍数
#define ADC_REFERENCE_VOLT 3.2f    // 电流采样adc参考电压
#define ADC_BITS 4096.0f           // ADC精度
#define ADC_SCALE (ADC_REFERENCE_VOLT / ADC_BITS)
#define CURRENT_FILTER_ALPHA 0.9f  // 滤波系数

extern volatile uint16_t adc_raw_ia;
extern volatile uint16_t adc_raw_ic;
extern volatile uint8_t current_loop_enable;

void ADC_Calibration(uint16_t cnt);
float VoltageToCurrent(uint32_t ADCValue, uint16_t offset_adc);

#endif

