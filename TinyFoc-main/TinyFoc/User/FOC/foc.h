#ifndef __FOC__H
#define __FOC__H

#include "stm32f4xx_hal.h"
#include "fast_trig.h"
#include <stdbool.h>
#include "utils.h"
#include "motor.h"
#include "main.h"
#include "tim.h"

void foc_forward(float d, float q, float angle_el);

#endif

