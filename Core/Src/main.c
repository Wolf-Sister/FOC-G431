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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_MODE_OPENLOOP  0   /* 0=closed-loop torque, 1=open-loop vector test */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Open-loop test globals — accessible from TIM callback */
uint8_t  test_phase  = 0;   /* 0=wait cal, 1=openloop test, 2=closed loop */
float    test_Uq     = 0.5f;
float    test_angle  = 0.0f;
float    test_speed  = 0.003f;   /* angle step per 20kHz ISR */

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
  /* USER CODE BEGIN 2 */
  char buf[128];
  uint32_t last_print_time = 0;

  UART2_SendString("========================================\r\n");
  UART2_SendString("  STM32G431 FOC v1 - Current Loop FOC\r\n");
  UART2_SendString("========================================\r\n");

  /* 1. Init DWT for PID microsecond timing */
  DWT_Init();

  /* 2. Init encoder sensor state */
  AS5047P_Sensor_Init(&AngleSensor);

  /* 3. Kick off first DMA encoder read (pipelined: each Update reads previous result) */
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

  UART2_SendString("[FOC] Waiting for current calibration...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* --- Phase 0: calibration done → choose path based on TEST_MODE_OPENLOOP --- */
    if (motor_current.Calibrated == 1 && test_phase == 0)
    {
      /* Log calibration result */
      char log_buf[128];
      sprintf(log_buf,
              "[FOC] Calibration Done! OffsetA: %.2f, OffsetC: %.2f\r\n",
              motor_current.Offset_A, motor_current.Offset_C);
      UART2_SendString(log_buf);

#if TEST_MODE_OPENLOOP
      test_phase = 1;
      UART2_SendString("[FOC] Open-loop vector test running...\r\n");
#else
      /* Sensor alignment */
      foc_alignSensor(4.0f);  /* stronger alignment to overcome cogging */

      /* Init motor state & PID */
      motor_control_parm_init();
      //             tor_p   tor_i   vel_p   vel_i   pos_p
motor_pid_init(0.05f,  5.0f,   0.0f,   0.0f,   0.0f);
      set_motor_mode(MOTOR_TORQUE);
      motor_control.set_torque = 0.2f;  /* 0.2A barely moves motor; 1A needed */

      /* current_loop_enable already set inside foc_alignSensor() */
      test_phase = 2;

      UART2_SendString("[FOC] Current Loop ACTIVE - Torque Mode\r\n");
#endif
    }

    /* --- Phase 1: open-loop test (set TEST_MODE_OPENLOOP=1 to use) --- */
    if (test_phase == 1)
    {
      if (HAL_GetTick() - last_print_time >= 100)
      {
        last_print_time = HAL_GetTick();
        float elec_deg = test_angle * 57.29578f;
        sprintf(buf,
          "Elec:%5.1f | Ia:%+.3f Ib:%+.3f Ic:%+.3f | Sum:%.3f\r\n",
          elec_deg,
          motor_control.IphA, motor_control.IphB, motor_control.IphC,
          motor_control.IphA + motor_control.IphB + motor_control.IphC);
        UART2_SendString(buf);
      }
    }

    /* --- Phase 2: closed-loop telemetry --- */
    if (test_phase == 2)
    {
      if (HAL_GetTick() - last_print_time >= 100)
      {
        last_print_time = HAL_GetTick();
        float mech_deg = AS5047P_GetAngle(&AngleSensor) * 57.29578f;
        float elec_deg = _normalizeAngle(11.0f * AS5047P_GetAngle(&AngleSensor)
                                         - motor_control.zero_elec_angle) * 57.29578f;
        sprintf(buf,
          "Mech:%5.1f Elec:%5.1f | Ia:%+.3f Ib:%+.3f Ic:%+.3f | "
          "Id:%.3f Iq:%.3f Ref:%.3f | Vd:%.2f Vq:%.2fV\r\n",
          mech_deg, elec_deg,
          motor_control.IphA, motor_control.IphB, motor_control.IphC,
          motor_control.id_meas, motor_control.iq_meas, motor_control.set_torque,
          motor_control.id_set, motor_control.iq_set);
        UART2_SendString(buf);
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
  * @brief  💡 双 ADC 注入同步采集完成中断回调函数 (由 TIM1_TRGO 硬件高频触发, 20kHz)
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint32_t calib_sum_a = 0;
    static uint32_t calib_sum_c = 0;
    static uint16_t calib_cnt    = 0;
    static uint8_t  cl_divider   = 0;  /* software divider for 20kHz→5kHz */
    static uint8_t  enc_divider  = 0;  /* software divider for 20kHz→1kHz */

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

            /* 4. Sync to motor_control & run current loop at 5 kHz */
            foc_sync_phase_currents();

            cl_divider++;
            if (cl_divider >= 4)   /* 20kHz / 4 = 5kHz */
            {
                cl_divider = 0;
                if (current_loop_enable) {
                    foc_current_loop();
                }
            }

            /* 5. Encoder sensor update at 5 kHz (20kHz / 4) */
            enc_divider++;
            if (enc_divider >= 4)
            {
                enc_divider = 0;
                AS5047P_Sensor_Update(&AngleSensor);
            }
        }
    }
}

/**
  * @brief  💡 TIM1 溢出/更新中断回调函数 (20kHz)
  *         Phase 1: open-loop rotating voltage vector test
  *         Phase 2: closed-loop current control (via ADC callback)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    /* During sensor alignment, do NOT overwrite PWM with rotating vector */
    if (alignment_in_progress) return;

    if (motor_current.Calibrated == 1 && current_loop_enable == 0)
    {
      if (test_phase == 1)
      {
        /* Open-loop rotating voltage vector — 20kHz angle update */
        test_angle += test_speed;
        if (test_angle > _2PI) test_angle -= _2PI;
        foc_forward(0.0f, test_Uq, test_angle);
      }
      else
      {
        /* Legacy open-loop SVPWM debug */
        motor_ctrl.Angle += motor_ctrl.Speed;
        if (motor_ctrl.Angle > _2PI) motor_ctrl.Angle -= _2PI;
        SVPWM_Update(motor_ctrl.Ud, motor_ctrl.Uq, motor_ctrl.Angle, motor_ctrl.Period);
      }
    }
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
