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
#include "fdcan.h"
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
#include "encoder_bench.h"
#include "foc.h"
#include "pid.h"
#include "uart_tx.h"
#include "utils.h"
#include "vofa.h"
#include "ina219.h"
#include "can.h"
#include "motor_supervisor.h"
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
uint8_t  test_phase  = 0;   /* 0=auto calibration, 1=stopped/fault, 2=ready disabled */

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

void App_CalibrationPoll(void)
{
  CAN_Poll();
#if APP_UART_ENABLE
  UART_TX_Pump();
#endif
}

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
  MX_FDCAN1_Init();
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

  DWT_Init();
  control_pid_init();
  motor_control_parm_init();
  MotorSupervisor_Init(HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2() ^ dwt_get_cycles(),
                       HAL_GetTick(), APP_AUTO_CALIBRATE != 0U);
  if (CAN_Init() != HAL_OK) {
    UART2_SendString("[CAN] ERROR: service not started\r\n");
    MotorSupervisor_StopISR();
  }
  INA219_ServiceInit();

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
     *    TIM2 encoder ISR sampling frequency is configured independently of TIM1. */
    (void)AS5047P_DMA_StartRequest();

    /* 4. Keep the gate driver disabled while calibrating current offsets. */
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
    Motor_Current_Calibration();

    /* 5. Start TIM1 ~32 kHz 3-phase PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    /* PWM starts the ADC trigger timer; offsets are sampled with outputs off. */
    Motor_HardwareDisable();

    /* 6. Start dual-ADC injected conversion (slave first, then master) */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

    /* 7. Start TIM1 update interrupt @ ~16 kHz → TRGO triggers ADC chain */
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler();

    UART2_SendString("[FOC] Waiting for current calibration...\r\n");
  }

  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) Error_Handler();

  /* 8. Start UART2 DMA RX for VOFA+ command reception */
#if APP_UART_ENABLE
  VOFA_InitRx();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MotorSupervisor_MainProgress(HAL_GetTick());
    INA219_ServicePoll(HAL_GetTick());
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

    CAN_Poll();

    /* Process incoming VOFA+ commands (PC → MCU) */
#if APP_UART_ENABLE
    VOFA_ProcessCmd();
    EncoderBench_Poll();
    VOFA_CheckCommandTimeout();
#endif

    /* Calibration progresses in TIM3 without blocking the main loop. */
    {
      static uint8_t offsetsReported, readyReported;
      CAN_NodeStatus status = {0};
      MotorSupervisor_ReadStatus(&status, HAL_GetTick());
      if (motor_current.Calibrated && !offsetsReported) {
        char log[100];
        (void)snprintf(log, sizeof(log), "[FOC] Calibration Done! OffsetA: %.2f, OffsetC: %.2f\r\n",
                       motor_current.Offset_A, motor_current.Offset_C);
        UART2_SendString(log); offsetsReported = 1U;
      }
      test_phase = status.state == CAN_STATE_READY || status.state == CAN_STATE_ENABLED ? 2U : 0U;
      if (motor_electrical_calibrated && !readyReported) {
        char log[100];
        (void)snprintf(log, sizeof(log), "[FOC] Zero elec angle = %.3f rad\r\n[FOC] Ready: output disabled; CAN Iq stage 2\r\n",
                       motor_control.zero_elec_angle);
        UART2_SendString(log); readyReported = 1U;
      }
    }

    /* --- Phase 2: existing UART telemetry, with the power stage disabled --- */
#if APP_UART_ENABLE
    if (test_phase == 2 && !EncoderBench_IsActive())
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
  * @brief  Dual-ADC injected conversion complete callback (triggered by TIM1_TRGO @ ~16kHz).
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
        CAN_OnCurrentSample();
    }
}

/**
  * @brief  TIM period elapsed callback
  *         - TIM1 update/TRGO (~16 kHz, PWM carrier ~32 kHz): safety no-op
  *           (FOC runs in ADC ISR)
  *         - TIM2: AS5047P encoder SPI read + angle/velocity → cache
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    foc_check_sampling_progress();
  }
  else if (htim->Instance == TIM2)
  {
    /* ── Encoder TIM2 update ── */
    /* ① SPI read raw angle → parity check → angle unwrap → velocity */
    if (AS5047P_Sensor_Update(&AngleSensor))
    {
      AS5047P_EncoderCache_Publish(&AngleSensor);
    }
  }
  else if (htim->Instance == TIM3)
  {
    foc_check_sampling_progress();
    MotorSupervisor_Tick(HAL_GetTick());
    CAN_Tick1kHz();
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
