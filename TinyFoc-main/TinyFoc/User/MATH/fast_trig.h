#ifndef __FAST_TRIG_H
#define __FAST_TRIG_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <stdint.h>

float fast_sin_f32(float x);
float fast_cos_f32(float x);
void fast_trig_init(void);

#endif
