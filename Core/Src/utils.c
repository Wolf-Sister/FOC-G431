/**
  ******************************************************************************
  * @file    utils.c
  * @brief   FOC utility implementations
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
static uint32_t cpu_freq_mhz = 0;

/* --------------------------------------------------------------------------*/
/**
  * @brief  Initialize DWT cycle counter for microsecond timing
  */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT      = 0;
    cpu_freq_mhz     = SystemCoreClock / 1000000;
}

/**
  * @brief  Get elapsed microseconds since DWT_Init (wraps ~70s on 170MHz)
  */
unsigned long dwt_get_micros(void)
{
    return DWT->CYCCNT / cpu_freq_mhz;
}

/**
  * @brief  First-order low-pass filter
  * @param  new_val  new sample
  * @param  alpha    smoothing factor (0～1, smaller = more filtering)
  * @retval filtered value
  */
float lowPassFilter(float new_val, float alpha)
{
    static float filtered = 0.0f;
    filtered = alpha * new_val + (1.0f - alpha) * filtered;
    return filtered;
}

/**
  * @brief  Normalize angle to [0, 2*PI)
  */
float _normalizeAngle(float angle)
{
    float a = fmodf(angle, _2PI);
    return (a < 0) ? a + _2PI : a;
}
