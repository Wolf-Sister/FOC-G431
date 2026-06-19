/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for AS5047P DMA Testing
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
#include <string.h>
#include <stdio.h>
#include "as5047p.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOC_LPF_ALPHA  0.15f  /* 一阶低通滤波系数 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define PI 3.1415926535f
#define SQRT3 1.7320508075f

typedef struct {
    float Uq;          // Q轴电压 (开环下代表电压幅值/转矩)
    float Ud;          // D轴电压 (开环设为0)
    float Angle;       // 电角度 (0 ~ 2*PI)
    float Speed;       // 步进速度 (决定电机转速，每次中断累加的弧度)
    uint32_t Period;   // PWM 周期 (ARR 寄存器的值 = 4250)
} OpenLoop_Ctrl_t;

// 初始化开环控制器
OpenLoop_Ctrl_t motor_ctrl = {
    .Uq = 0.20f,        // 初始电压幅值（如果太热可以微调至0.2f~0.25f）
    .Ud = 0.0f,
    .Angle = 0.0f,
    .Speed = 0.002f,  // 在20kHz中断里，每次累加0.0015弧度（对应11极对数电机约为26 RPM）
    .Period = 4250
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART2_SendString(const char *str);
float FOC_GetSmoothAngle(void);
void SVPWM_Update(float Ud, float Uq, float angle, uint32_t period);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SVPWM_Update(float Ud, float Uq, float angle, uint32_t period)
{
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    float Ualpha = Ud * cos_angle - Uq * sin_angle;
    float Ubeta  = Ud * sin_angle + Uq * cos_angle;

    uint8_t sector = 0;
    float v1 = Ubeta;
    float v2 = (SQRT3 * Ualpha - Ubeta) / 2.0f;
    float v3 = (-SQRT3 * Ualpha - Ubeta) / 2.0f;

    if (v1 > 0) sector += 1;
    if (v2 > 0) sector += 2;
    if (v3 > 0) sector += 4;

    switch (sector) {
        case 3: sector = 1; break;
        case 1: sector = 2; break;
        case 5: sector = 3; break;
        case 4: sector = 4; break;
        case 6: sector = 5; break;
        case 2: sector = 6; break;
        default: return;
    }

    float Tlow = (float)period; 
    float X = SQRT3 * Tlow * Ubeta;
    float Y = (3.0f * Ualpha + SQRT3 * Ubeta) * Tlow / 2.0f;
    float Z = (-3.0f * Ualpha + SQRT3 * Ubeta) * Tlow / 2.0f;

    float t1 = 0.0f, t2 = 0.0f;
    switch (sector) {
        case 1: t1 = -Z; t2 =  X; break;
        case 2: t1 =  Y; t2 =  Z; break;
        case 3: t1 =  X; t2 = -Y; break;
        case 4: t1 =  Z; t2 = -X; break;
        case 5: t1 = -Y; t2 = -Z; break;
        case 6: t1 = -X; t2 =  Y; break;
    }

    float sum = t1 + t2;
    if (sum > Tlow) {
        t1 = t1 * Tlow / sum;
        t2 = t2 * Tlow / sum;
    }

    float ta = (Tlow - t1 - t2) / 4.0f;
    float tb = ta + t1 / 2.0f;
    float tc = tb + t2 / 2.0f;

    uint16_t ccr1 = 0, ccr2 = 0, ccr3 = 0;
    switch (sector) {
        case 1: ccr1 = (uint16_t)ta; ccr2 = (uint16_t)tb; ccr3 = (uint16_t)tc; break;
        case 2: ccr1 = (uint16_t)tb; ccr2 = (uint16_t)ta; ccr3 = (uint16_t)tc; break;
        case 3: ccr1 = (uint16_t)tc; ccr2 = (uint16_t)ta; ccr3 = (uint16_t)tb; break;
        case 4: ccr1 = (uint16_t)tc; ccr2 = (uint16_t)tb; ccr3 = (uint16_t)ta; break;
        case 5: ccr1 = (uint16_t)tb; ccr2 = (uint16_t)tc; ccr3 = (uint16_t)ta; break;
        case 6: ccr1 = (uint16_t)ta; ccr2 = (uint16_t)tc; ccr3 = (uint16_t)tb; break;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}

/**
  * @brief  获取抗噪平滑角度，包含过零处理与一阶低通滤波
  * @retval 0.0 ~ 360.0 度的平滑角度
  */
float FOC_GetSmoothAngle(void)
{
  static float filtered_angle = 0.0f;
  static uint8_t is_first = 1;
  
  /* 1. 从底层驱动安全获取热乎的 14b DMA 原始数据 */
  uint16_t dma_raw = AS5047P_DMA_GetAngleCallback();
  
  /* 如果读取发生严重故障，直接维持上一次的滤波角度，保护控制环路 */
  if (dma_raw == 0xFFFFU) return filtered_angle; 
  
  /* 2. 将 14b 原始数据转为浮点角度 */
  float current_angle = (float)dma_raw * (360.0f / 16384.0f);
  
  if (is_first)
  {
    filtered_angle = current_angle;
    is_first = 0;
    return filtered_angle;
  }
  
  /* 3. 过零点软处理（防止在 360° 和 0° 交界处滤波器误判导致数据剧烈震荡） */
  float diff = current_angle - filtered_angle;
  if (diff > 180.0f)  diff -= 360.0f;
  if (diff < -180.0f) diff += 360.0f;
  
  /* 4. 一阶低通滤波计算 */
  filtered_angle += FOC_LPF_ALPHA * diff;
  
  /* 5. 限制范围在 0 ~ 360 度之间 */
  if (filtered_angle >= 360.0f) filtered_angle -= 360.0f;
  if (filtered_angle < 0.0f)    filtered_angle += 360.0f;
  
  return filtered_angle;
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
  /* USER CODE BEGIN 2 */
  char buf[128];
  uint32_t last_print_time = 0; // 用于串口低频打印的计时器

  UART2_SendString("========================================\r\n");
  UART2_SendString("  STM32G431 FOC v1 - TIM Interrupt SVPWM\r\n");
  UART2_SendString("========================================\r\n");

  /* 初始化 AS5047P 磁编码器 */
  UART2_SendString("[AS5047P] Initializing DMA Mode...\r\n");
  AS5047P_Init();
  UART2_SendString("[AS5047P] Init complete. Starting test loop.\r\n\r\n");

  UART2_SendString("Starting TIM1 Interrupt Driven SVPWM Run:\r\n");
  UART2_SendString("----------------------------------------\r\n");

  // 我们只需要启动 TIM1 的三路正相 PWM 即可：
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // 开启高级定时器主输出使能
  __HAL_TIM_MOE_ENABLE(&htim1);

  // 💡 核心：清除可能存在的待处理中断标志位，并正式启动 TIM1 更新中断
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ------------------ 磁编码器读取部分 ------------------ */
    // 如果需要开启编码器测试，将下面的一句 if (0) 改为 if (1) 即可
    if (1)
    {
      /* 1. 异步发起一次 DMA 角度采集请求 */
      AS5047P_DMA_StartRequest();
    
      /* 2. 获取平滑后的角度 */
      float smooth_angle = FOC_GetSmoothAngle();
    
      /* 3. 使用非阻塞的 HAL_GetTick() 限制打印频率为 100ms 一次，绝不干扰开环中断 */
      if (HAL_GetTick() - last_print_time >= 100)
      {
        last_print_time = HAL_GetTick();
        sprintf(buf, "Smooth Angle: %6.2f deg\r\n", smooth_angle);
        UART2_SendString(buf);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 💡 核心优化：这里已经完全没有任何延时和开环处理，CPU 在此静静等待中断或处理串口
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
  * @brief  通过 UART2 发送字符串（阻塞方式）
  * @param  str: 要发送的字符串指针
  * @retval None
  */
void UART2_SendString(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief  💡 TIM1 溢出/更新中断回调函数 (20kHz 高频触发)
  * @param  htim: 定时器句柄指针
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    // 1. 累加微小的电角度（保持极细腻的步伐过渡）
    motor_ctrl.Angle += motor_ctrl.Speed;
    if (motor_ctrl.Angle > 2.0f * PI) {
        motor_ctrl.Angle -= 2.0f * PI;
    }

    // 2. 严格按 20kHz 频率刷新三相 SVPWM，带转子平滑荡过死区畸变区
    SVPWM_Update(motor_ctrl.Ud, motor_ctrl.Uq, motor_ctrl.Angle, motor_ctrl.Period);
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
