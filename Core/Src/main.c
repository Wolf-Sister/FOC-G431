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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "as5047p.h"
#include "as5047p_ext.h"
#include "foc.h"
#include "pid.h"
#include "uart_tx.h"
#include "utils.h"
#include "vofa.h"
#include "ina219.h"
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

/* 外设就绪标志 — Error_Handler() 据此判断哪些寄存器可以安全操作。
 * 必须在 MX_TIM1_Init() / MX_USART2_UART_Init() 的 USER CODE 区置位，
 * 否则 CubeMX 重新生成后本文件的关断逻辑会误判外设状态。 */
volatile uint8_t htim1_ready  = 0U;
volatile uint8_t huart2_ready = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void App_ReportEncoderDiagnostics(void)
{
#if APP_UART_ENABLE
  as5047p_diagnostics_t snapshot;
  char log[128];
  AS5047P_GetDiagnostics(&snapshot);
  (void)snprintf(log, sizeof(log),
                 "[ENC] cause=%s dia=%04X mag=%04X spi=%u/%u err=%04X/%u tries=%u\r\n",
                 AS5047P_DiagnosticStatusName(snapshot.status),
                 (unsigned)snapshot.diaagc_raw, (unsigned)snapshot.magnitude_raw,
                 (unsigned)snapshot.diagnostic_spi_status, (unsigned)snapshot.magnitude_spi_status,
                 (unsigned)snapshot.error_raw, (unsigned)snapshot.error_valid, (unsigned)snapshot.attempts);
  UART2_SendString(log);
#endif
}

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
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  App_DMA_DisableUnusedUART();
#if APP_UART_ENABLE
  uint32_t last_print_time = 0;
  /* Initialize before the first startup log is queued. */
  UART_TX_Init();
#endif

  UART2_SendString("========================================\r\n");
  UART2_SendString("  STM32G431 FOC v1\r\n");
  UART2_SendString("========================================\r\n");

  /* INA219 母线电压/电流检测 — 母线无效时禁止使能功率级 */
  if (INA219_Init() != HAL_OK)
  {
    UART2_SendString("[INA219] ERROR: device not found on I2C1\r\n");
  }
  else
  {
    UART2_SendString("[INA219] OK: bus monitor ready\r\n");
  }

  /* 1. Init DWT for PID microsecond timing */
  DWT_Init();

  /* 2. Init encoder sensor state */
  AS5047P_Sensor_Init(&AngleSensor);
  AS5047P_Init();
  if (!AS5047P_CheckDiagnostics()) {
    motor_fault_trip(MOTOR_FAULT_ENCODER);
  } else {
    App_ReportEncoderDiagnostics();
  }

  if (motor_fault_reason == MOTOR_FAULT_NONE) {
    /* 4. Kick off first DMA encoder read (pipelined: each Update reads previous result).
     *    TIM2 encoder ISR @ 20kHz is started later by foc_alignSensor(). */
    (void)AS5047P_DMA_StartRequest();

    /* 4. Keep the gate driver disabled while calibrating current offsets. */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    Motor_Current_Calibration();

    /* 5. Start TIM1 40 kHz 3-phase PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 6. Start dual-ADC injected conversion (slave first, then master) */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

    /* 7. Start TIM1 update interrupt @ 20 kHz → TRGO triggers ADC chain */
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim1);

    UART2_SendString("[FOC] Waiting for current calibration...\r\n");
  }

  /* 8. Start UART2 DMA RX for VOFA+ command reception */
#if APP_UART_ENABLE
  VOFA_InitRx();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    foc_check_calibration_timeout();
    {
      static uint8_t fault_reported = 0U;
      if (motor_fault_reason != MOTOR_FAULT_NONE && !fault_reported) {
        char fault_log[128];
        if (motor_fault_reason == MOTOR_FAULT_ENCODER) App_ReportEncoderDiagnostics();
        (void)snprintf(fault_log, sizeof(fault_log),
                       "[FAULT] reason=%u tick=%lu raw=%u/%u stage=%u/%u; reset required\r\n",
                       (unsigned)motor_fault_reason, (unsigned long)motor_fault_snapshot.tick_ms,
                       (unsigned)motor_fault_snapshot.raw_a, (unsigned)motor_fault_snapshot.raw_c,
                       (unsigned)motor_fault_snapshot.was_aligning,
                       (unsigned)motor_fault_snapshot.was_running);
        UART2_SendString(fault_log);
        fault_reported = 1U;
        test_phase = 1U;
      }
    }
    /* UART2 发送泵 — 必须放在循环最前面且不做节流。
     * 它自身只在 DMA 空闲时工作，开销仅一次状态判断；若放在遥测节流之后，
     * 队列排空速率会被遥测周期限制。 */
#if APP_UART_ENABLE
    UART_TX_Pump();
#endif

    /* Slow bus supervision (5 Hz), not a substitute for fast phase protection. */
    {
      static uint32_t last_ina_time = 0;
      if (!ina219_bus_valid || HAL_GetTick() - last_ina_time >= 200U)
      {
        last_ina_time = HAL_GetTick();

        float vbus = 0.0f;
        float ibus = 0.0f;

        if (INA219_ReadBusVoltage(&vbus) == HAL_OK)
        {
          if (!isfinite(vbus) || vbus < FOC_BUS_MIN_VALID_V || vbus > FOC_BUS_MAX_VALID_V) {
            ina219_bus_valid = 0U;
            motor_fault_trip(MOTOR_FAULT_BUS);
          } else {
            motor_config.voltage_supply = vbus;
          }
        } else {
          motor_fault_trip(MOTOR_FAULT_BUS);
        }
        if (INA219_ReadCurrent(&ibus) == HAL_OK)
        {
          ina219_ibus = ibus;
        } else {
          ina219_ibus = NAN;
        }
      }
    }

    /* Process incoming VOFA+ commands (PC → MCU) */
#if APP_UART_ENABLE
    VOFA_ProcessCmd();
    VOFA_CheckCommandTimeout();
#endif

    /* --- Phase 0: calibration done → sensor alignment → closed loop --- */
    if (motor_current.Calibrated == 1 && test_phase == 0 && motor_fault_reason == MOTOR_FAULT_NONE)
    {
      /* Log calibration result */
      char log_buf[128];
      (void)snprintf(log_buf, sizeof(log_buf),
                     "[FOC] Calibration Done! OffsetA: %.2f, OffsetC: %.2f\r\n",
                     motor_current.Offset_A, motor_current.Offset_C);
      UART2_SendString(log_buf);

      /* Enable the gate driver before alignment. */
      if (!foc_enable_driver() || foc_alignSensor() != HAL_OK)
      {
        /* Keep startup failures safe: disable the gate driver and do not
         * enter closed loop with an invalid zero angle. */
        motor_fault_trip(MOTOR_FAULT_ALIGN_TIMEOUT);
        test_phase = 1;
        continue;
      }

      encoder_cache_t encoder_wait = {0};

      /* 等待编码器缓存被填充 (至少 2 次 TIM2 更新 ≈ 200 us)。
       * 必须有超时: 原实现是无界 do-while，若编码器 DMA 始终不产出有效帧，
       * 主循环会永久卡在这里，而 TIM1 仍在输出 PWM。 */
      bool encoder_ready = false;
      uint32_t encoder_wait_start = HAL_GetTick();
      do {
        (void)AS5047P_EncoderCache_Read(&encoder_wait);
        if (motor_fault_reason != MOTOR_FAULT_NONE) break;
        if (encoder_wait.data_valid && !encoder_wait.stale && encoder_wait.update_count >= 2U) {
          encoder_ready = true;
          break;
        }
      } while (HAL_GetTick() - encoder_wait_start < 100U);   /* 100 ms 超时 */

      if (!encoder_ready)
      {
        /* 编码器无响应: 关断功率级并停在安全状态，不允许进入闭环 */
        UART2_SendString("[FOC] ERROR: encoder not responding, aborting\r\n");
        motor_fault_trip(MOTOR_FAULT_ENCODER);
        test_phase = 1;
        continue;
      }

      /* Init motor state & PID */
      motor_control_parm_init();
      control_pid_init();

      motor_control.set_torque = 0.0f;

      motor_control.id_target  = 0.0f;
      motor_control.set_speed  = 0.0f;
      /* Enable command handling and the ISR only after state initialization. */
      if (!foc_enable_control()) {
        motor_fault_trip(MOTOR_FAULT_ENCODER);
        test_phase = 1U;
        continue;
      }
      test_phase = 2;

      /* Start TIM3 position-loop timer (1 kHz). ISR returns immediately
       * unless mode == MOTOR_POSITION. */
      HAL_TIM_Base_Start_IT(&htim3);

    }

    /* --- Phase 2: closed-loop telemetry via VOFA+ JustFloat --- */
#if APP_UART_ENABLE
    if (test_phase == 2)
    {
      if (HAL_GetTick() - last_print_time >= 10)
      {
        last_print_time = HAL_GetTick();

        /* Channel layout for VOFA+:
         *   [0] id_target       - D-axis target current (A)
         *   [1] id_meas         - D-axis actual current (A)
         *   [2] iq_target       - Q-axis target current (A) = set_torque
         *   [3] iq_meas         - Q-axis actual current (A)
         *   [4] velocity_raw    - Sliding-window velocity (rad/s)
         *   [5] velocity_filt   - Filtered velocity used by speed PI (rad/s)
         *   [6] speed_setpoint  - Speed setpoint (rad/s)
         *   [7] speed_kp_active - Active scheduled speed P gain
         *   [8] status_flag     - Command synchronization flag
         *   [9] mode            - Control mode (0=torque, 1=speed, 2=pos)
         *   [10] position_target - Absolute multi-turn position target (rad)
         *   [11] position_meas   - Measured multi-turn position (rad)
         *   [12] bus_voltage    - Bus voltage from INA219 (V)
         *   [13] bus_current    - Bus current from INA219 (A)
         */
        float vofa_data[14] = {
            motor_control.id_target,
            motor_control.id_meas,
            motor_control.set_torque,
            motor_control.iq_meas,
            motor_control.vel_raw,
            motor_control.vel_meas,
            motor_control.set_speed,
            motor_control.spd_kp_active,
            (float)motor_control.status_flag,
            (float)motor_control.mode,
            motor_control.set_position,
            motor_control.pos_meas,
            ina219_vbus,      /* [12] 母线电压 (V) */
            ina219_ibus       /* [13] 母线电流 (A) */
        };
        VOFA_SendData(vofa_data, 14);
        motor_control.status_flag = 0;  /* clear after TX */
      }
    }

#endif
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
    if (hadc->Instance == ADC1)
    {
        /* 1. Read dual-ADC synchronized raw values */
        uint16_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t raw_c = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        foc_process_current_sample(raw_a, raw_c);
    }
}

/**
  * @brief  TIM period elapsed callback
  *         - TIM1 update/TRGO (20 kHz, PWM carrier 40 kHz): safety no-op
  *           (FOC runs in ADC ISR)
  *         - TIM2 (20 kHz): AS5047P encoder SPI read + angle/velocity → cache
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    foc_check_sampling_progress();
  }
  else if (htim->Instance == TIM2)
  {
    /* ── 20 kHz encoder update ── */
    /* ① SPI read raw angle → parity check → angle unwrap → velocity */
    if (AS5047P_Sensor_Update(&AngleSensor))
    {
      AS5047P_EncoderCache_Publish(&AngleSensor);
    }
  }
  else if (htim->Instance == TIM3)
  {
    /* ── 1 kHz Position loop ── */
    foc_position_loop();
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
  __disable_irq();

  /* 1. 关断功率级 — 无条件执行，不依赖任何外设初始化状态。
   *    BSRR 高 16 位写 1 表示对应引脚复位(输出低)，直接写寄存器以覆盖
   *    "HAL_GPIO 尚未初始化" 的早期失败场景。
   *    PB13/PB14/PB15 是 DRV8323H 3x PWM 模式的 INLx 绑定脚，不得触碰。 */
  Motor_HardwareDisable();

  /* Stop the trigger timer after the shared immediate hardware shutdown. */
  if (htim1_ready != 0U)
  {
    TIM1->CR1  &= ~TIM_CR1_CEN;
  }

  /* 3. 尽力上报一次 (环形缓冲非阻塞，满则丢弃) */
  if (huart2_ready != 0U)
  {
    (void)UART_TX_PutString("[FATAL] Error_Handler: power stage disabled\r\n");
    UART_TX_Pump();
  }

  /* IWDG is not configured; remain disabled until an external reset. */
  for (;;)
  {
    __NOP();
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
