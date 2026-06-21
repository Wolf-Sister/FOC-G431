/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

/* USER CODE BEGIN 0 */
#include "stm32g4xx_ll_cordic.h"
/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* One-time CORDIC config for FOC sin/cos pipeline:
   *   Cosine function (generates both sin + cos when NBREAD=2)
   *   6 cycles × 4 iter/cycle = 24 iterations → ~6e-8 precision (exceeds float32)
   *   Q1.31 I/O, 1×32-bit write (angle), 2×32-bit read (cos then sin)
   *   IEN enabled → CORDIC ISR fires on result ready
   */
  LL_CORDIC_Config(CORDIC,
                   LL_CORDIC_FUNCTION_COSINE,
                   LL_CORDIC_PRECISION_6CYCLES,
                   LL_CORDIC_SCALE_0,
                   LL_CORDIC_NBWRITE_1,
                   LL_CORDIC_NBREAD_2,
                   LL_CORDIC_INSIZE_32BITS,
                   LL_CORDIC_OUTSIZE_32BITS);
  LL_CORDIC_EnableIT(CORDIC);

  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CORDIC interrupt Init */
    HAL_NVIC_SetPriority(CORDIC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CORDIC_IRQn);
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC interrupt Deinit */
    HAL_NVIC_DisableIRQ(CORDIC_IRQn);
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
