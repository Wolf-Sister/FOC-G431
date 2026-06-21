/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for AS5047P and Dual ADC FOC Testing
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "as5047p.h"
#include "as5047p_ext.h"
#include "foc.h"
#include "pid.h"
#include "utils.h"
#include "vofa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VEL_LIMIT      600.0f   /* Velocity clamp ±600 rad/s (~5700 RPM)       */
#define VEL_LPF_ALPHA  0.3f     /* 1st-order LPF coefficient (fc≈2.5Hz @100Hz) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Control state globals — accessible from TIM/ADC callbacks */
uint8_t  test_phase  = 0;   /* 0=wait cal, 1=openloop, 2=closed loop */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  uint32_t last_print_time = 0;

  UART2_SendString("========================================\r\n");
  UART2_SendString("  STM32G431 FOC v1\r\n");
  UART2_SendString("========================================\r\n");

  /* 1. Init DWT for PID microsecond timing */
  DWT_Init();

  /* 2. Init encoder sensor state */
  AS5047P_Sensor_Init(&AngleSensor);

  /* 3. Kick off first DMA encoder read (pipelined: each Update reads previous result).
   *    TIM2 encoder ISR @ 10kHz is started later by foc_alignSensor(). */
  AS5047P_DMA_StartRequest();

  /* 4. Current-sensor offset calibration (runs asynchronously in ADC ISR) */
  Motor_Current_Calibration();

  /* 5. Start TIM1 3-phase PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* 6. Start dual-ADC injected conversion (slave first, then master) */
  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  /* 7. Start TIM1 update interrupt → TRGO triggers ADC chain */
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);

  /* 8. Start UART2 DMA RX for VOFA+ command reception */
  VOFA_InitRx();

  UART2_SendString("[FOC] Waiting for current calibration...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Process incoming VOFA+ commands (PC → MCU) */
    VOFA_ProcessCmd();

    /* --- Phase 0: calibration done → sensor alignment → closed loop --- */
    if (motor_current.Calibrated == 1 && test_phase == 0)
    {
      /* Log calibration result */
      char log_buf[128];
      sprintf(log_buf,
              "[FOC] Calibration Done! OffsetA: %.2f, OffsetC: %.2f\r\n",
              motor_current.Offset_A, motor_current.Offset_C);
      UART2_SendString(log_buf);

      /* Sensor alignment (starts TIM2 encoder ISR internally) */
      foc_alignSensor(4.0f);  /* stronger alignment to overcome cogging */

      /* Wait for encoder cache to be populated (at least 2 TIM2 updates ≈ 200 us) */
      while (encoder_cache.update_count < 2) { /* spin */ }

      /* Init motor state & PID */
      motor_control_parm_init();
              motor_pid_init(1.4850f, 371.25f,   /* Iq: P, I */
                             1.4850f, 371.25f   /* Id: P, I */
                            );
              speed_pid_init(0.0120f, 0.0050f);        /* Speed: P, I */

      motor_control.set_torque = 0.0f;

      /* current_loop_enable already set inside foc_alignSensor() */
      test_phase = 2;

    }

    /* --- Phase 2: closed-loop telemetry via VOFA+ JustFloat --- */
    if (test_phase == 2)
    {
      if (HAL_GetTick() - last_print_time >= 10)
      {
        last_print_time = HAL_GetTick();

        /* Channel layout for VOFA+ / Python tuning script:
         *   [0] id_target      - D-axis target current (A)
         *   [1] id_meas        - D-axis actual current (A)
         *   [2] iq_target      - Q-axis target current (A) = set_torque
         *   [3] iq_meas        - Q-axis actual current (A)
         *   [4] vd_cmd         - D-axis voltage command (V)
         *   [5] vq_cmd         - Q-axis voltage command (V)
         *   [6] velocity       - Filtered mechanical velocity (rad/s)
         *   [7] status_flag    - Step-sync flag (1=cmd received)
         *   [8] speed_setpoint - Speed setpoint (rad/s)
         *   [9] mode           - Control mode (0=torque, 1=speed)
         */
        float vofa_data[10] = {
            motor_control.id_target,
            motor_control.id_meas,
            motor_control.set_torque,
            motor_control.iq_meas,
            motor_control.id_set,
            motor_control.iq_set,
            motor_control.vel_meas,
            (float)motor_control.status_flag,
            motor_control.set_speed,
            (float)motor_control.mode
        };
        VOFA_SendData(vofa_data, 10);
        motor_control.status_flag = 0;  /* clear after TX */
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Dual-ADC injected conversion complete callback (triggered by TIM1_TRGO @ 20kHz).
  *         Handles current offset calibration, phase current computation, encoder update,
  *         and closed-loop FOC current control.
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t calib_sum_a = 0;
    static uint32_t calib_sum_c = 0;
    static uint16_t calib_cnt    = 0;
    if (hadc->Instance == ADC1)
    {
        /* 1. Read dual-ADC synchronized raw values */
        uint16_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t raw_c = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        /* 2. Calibration state machine (first 2000 samples → 0.1s @ 20kHz) */
        if (motor_current.Calibrated == 0)
        {
            calib_sum_a += raw_a;
            calib_sum_c += raw_c;
            calib_cnt++;

            if (calib_cnt >= 2000)
            {
                motor_current.Offset_A   = (float)calib_sum_a / 2000.0f;
                motor_current.Offset_C   = (float)calib_sum_c / 2000.0f;
                motor_current.Calibrated = 1;
            }
        }
        else
        {
            /* 3. Convert raw ADC → phase current (A) */
            motor_current.Raw_A = raw_a;
            motor_current.Raw_C = raw_c;

            float current_diff_a = (float)motor_current.Raw_A - motor_current.Offset_A;
            float current_diff_c = (float)motor_current.Raw_C - motor_current.Offset_C;

            motor_current.I_A = current_diff_a * CURRENT_FACTOR;
            motor_current.I_C = current_diff_c * CURRENT_FACTOR;
            motor_current.I_B = -(motor_current.I_A + motor_current.I_C);

			      /* 1. Encoder updated by TIM2 ISR @ 10kHz — FOC reads from cache */

			      // 2. 同步电流数据
			      foc_sync_phase_currents();

			      // 3. 执行电流环 @ 20kHz
			      if (current_loop_enable) {
				      foc_current_loop();
			      }
        }
    }
}

/**
  * @brief  TIM period elapsed callback
  *         - TIM1 (20 kHz): PWM safety no-op (FOC runs in ADC ISR)
  *         - TIM2 (10 kHz): AS5047P encoder SPI read + angle/velocity → cache
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    /* During sensor alignment, do NOT overwrite PWM */
    if (alignment_in_progress) return;
  }
  else if (htim->Instance == TIM2)
  {
    /* ── 10 kHz encoder update ── */
    /* ① SPI read raw angle → parity check → angle unwrap → velocity */
    AS5047P_Sensor_Update(&AngleSensor);

    /* ② Publish results to volatile cache for FOC current loop (read-only) */
    encoder_cache.angle_raw       = AngleSensor.prev_angle;
    encoder_cache.velocity_rad_s  = AngleSensor.velocity_rad_s;
    encoder_cache.total_angle_rad = AngleSensor.total_angle;
    encoder_cache.update_count++;
    encoder_cache.data_valid      = 1;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
