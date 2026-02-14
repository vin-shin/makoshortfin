/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stm32g4xx_hal_dma.h"
#include "a1333.h"
#include "FOC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define ADC_VREF_MV 3300U
#define ADC_MAX_COUNTS 4095U
#define CURRENT_ZERO_MV 2500
#define CURRENT_SENSE_MV_PER_A 20
#define OPAMP_ADC_CHANNEL ADC_CHANNEL_12
#define FOC_POLE_PAIRS 20U
#define ENCODER_COUNTS_PER_REV 32768.0f
#define TWO_PI_F 6.28318530718f

/* A1333 CS pin on PA15 (for SPI3) */
#define A1333_CS_PORT GPIOA
#define A1333_CS_PIN GPIO_PIN_15

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t ia_zero_mv = CURRENT_ZERO_MV;
static uint32_t ib_zero_mv = CURRENT_ZERO_MV;
static uint32_t ic_zero_mv = CURRENT_ZERO_MV;

static volatile float iir_alpha = 0.15f;

static float ia_mv_f = 0.0f;
static float ib_mv_f = 0.0f;
static float ic_mv_f = 0.0f;
static float bus_mv_f = 0.0f;
static uint8_t filt_ready = 0;

static volatile uint32_t adc_dual_raw = 0;

/* A1333 Encoder Variables */
static A1333_Handle angle_sensor;
static uint16_t encoder_angle_raw = 0;
static float encoder_angle_deg = 0.0f;
static int16_t encoder_angle_prev = 0;
static float encoder_speed = 0.0f;  // rad/s
static uint32_t encoder_last_time = 0;

/* Motor commutation variables removed for clean slate */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Send(char* message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

void Filter_SetAlpha(float alpha)
{
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  iir_alpha = alpha;
}

float Filter_GetAlpha(void)
{
  return (float)iir_alpha;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t ReadAdcInjectedCounts(ADC_HandleTypeDef *hadc)
{
  if (HAL_ADCEx_InjectedStart(hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedPollForConversion(hadc, 10) != HAL_OK)
  {
    Error_Handler();
  }
  uint32_t value = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
  HAL_ADCEx_InjectedStop(hadc);
  return value;
}

static uint32_t AdcCountsToMv(uint32_t counts)
{
  return (counts * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static void AutoZeroCurrents(void)
{
  const uint32_t samples = 512U;
  uint64_t sum_ia = 0;
  uint64_t sum_ib = 0;

  for (uint32_t i = 0; i < samples; ++i)
  {
    uint32_t raw = adc_dual_raw;
    uint16_t ia_raw = (uint16_t)(raw & 0xFFFF);
    uint16_t ib_raw = (uint16_t)(raw >> 16);
    sum_ia += AdcCountsToMv(ia_raw);
    sum_ib += AdcCountsToMv(ib_raw);
    HAL_Delay(1);
  }

  ia_zero_mv = (uint32_t)(sum_ia / samples);
  ib_zero_mv = (uint32_t)(sum_ib / samples);
}

static void UpdateMovingAverage(uint32_t ia_mv, uint32_t ib_mv, uint32_t ic_mv, uint32_t bus_mv)
{
  if (!filt_ready)
  {
    ia_mv_f = (float)ia_mv;
    ib_mv_f = (float)ib_mv;
    ic_mv_f = (float)ic_mv;
    bus_mv_f = (float)bus_mv;
    filt_ready = 1U;
    return;
  }

  float alpha = (float)iir_alpha;
  ia_mv_f += alpha * ((float)ia_mv - ia_mv_f);
  ib_mv_f += alpha * ((float)ib_mv - ib_mv_f);
  ic_mv_f += alpha * ((float)ic_mv - ic_mv_f);
  bus_mv_f += alpha * ((float)bus_mv - bus_mv_f);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_OPAMP3_Init();
  /* USER CODE BEGIN 2 */
  UART_Send("Hello, Mako Shortfin!\n");
  UART_Send("System init\r\n");
    
    // Test with printf (after retargeting)
    printf("Clock: %lu Hz\r\n", SystemCoreClock);
  
  /* A1333 Encoder Initialization */
  printf("A1333 encoder init\r\n");

  FOC_Init();
  FOC_SetPolePairs(FOC_POLE_PAIRS);
  
  A1333_Status a1333_status = A1333_Init(&angle_sensor, &hspi3, A1333_CS_PORT, A1333_CS_PIN);
  if (a1333_status != A1333_OK)
  {
    printf("A1333 init failed! Status: %d\r\n", a1333_status);
  }
  else
  {
    printf("A1333 init OK\r\n");
    
    // Clear RST flag (always set after power-on)
    A1333_ClearErrors(&angle_sensor);
    A1333_ClearWarnings(&angle_sensor);
    
    // Wait for angle to become valid
    bool valid = false;
    for (int i = 0; i < 20 && !valid; i++)
    {
      A1333_IsAngleValid(&angle_sensor, &valid);
      HAL_Delay(10);
    }
    
    // Read initial angle
    if (A1333_ReadAngle15(&angle_sensor, &encoder_angle_raw, &encoder_angle_deg) == A1333_OK)
    {
      encoder_angle_prev = (int16_t)encoder_angle_raw;
      encoder_last_time = HAL_GetTick();
      printf("A1333 initial: %u raw, %.2f deg\r\n", encoder_angle_raw, encoder_angle_deg);
    }
    
    // Read diagnostics
    uint16_t status, errors;
    A1333_ReadStatus(&angle_sensor, &status);
    A1333_ReadErrors(&angle_sensor, &errors);
    printf("Status: 0x%04X, Errors: 0x%04X\r\n", status, errors);
  }

    uint32_t last_print_ms = 0;
    uint32_t last_encoder_ms = 0;

    if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&adc_dual_raw, 1) != HAL_OK)
    {
      Error_Handler();
    }
    if (hadc1.DMA_Handle != NULL)
    {
      __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
    }

    HAL_Delay(50);

    AutoZeroCurrents();

    printf("ADC dual-sampling @ 30kHz (TIM1_TRGO2)\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
     while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Read A1333 encoder continuously for smooth commutation */
    A1333_ReadAngle15(&angle_sensor, &encoder_angle_raw, &encoder_angle_deg);
    
    /* Calculate velocity every 50ms */
    if ((now - last_encoder_ms) >= 50U)
    {
      last_encoder_ms = now;
      
      // Calculate velocity
      int32_t delta = (int16_t)encoder_angle_raw - encoder_angle_prev;
      if (delta > 16384) delta -= 32768;
      else if (delta < -16384) delta += 32768;
      
      if (encoder_last_time > 0 && now > encoder_last_time)
      {
        uint32_t dt_ms = now - encoder_last_time;
        float new_speed = (float)delta * 0.000191747f / ((float)dt_ms * 0.001f);
        encoder_speed += (new_speed - encoder_speed) * 0.1f;
      }
      
      encoder_angle_prev = (int16_t)encoder_angle_raw;
      encoder_last_time = now;
    }

    /* Print combined status every 200ms */
    if ((now - last_print_ms) >= 200U)
    {
      last_print_ms = now;
      uint32_t raw = adc_dual_raw;
      uint16_t raw_ia = (uint16_t)(raw & 0xFFFF);
      uint16_t raw_ib = (uint16_t)(raw >> 16);
      uint32_t raw_opamp = ReadAdcInjectedCounts(&hadc1);

      uint32_t mv_ia = AdcCountsToMv(raw_ia);
      uint32_t mv_ib = AdcCountsToMv(raw_ib);
      int32_t ia_ma = ((int32_t)mv_ia - (int32_t)ia_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
      int32_t ib_ma = ((int32_t)mv_ib - (int32_t)ib_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
      int32_t ic_ma = -(ia_ma + ib_ma);
      int32_t mv_ic = (int32_t)ic_zero_mv + (ic_ma * CURRENT_SENSE_MV_PER_A) / 1000;
      if (mv_ic < 0) mv_ic = 0;
      if (mv_ic > (int32_t)ADC_VREF_MV) mv_ic = ADC_VREF_MV;

      uint32_t mv_opamp = AdcCountsToMv(raw_opamp);
      uint32_t mv_bus = mv_opamp * 21U;

      float mech_angle_rad = ((float)encoder_angle_raw * TWO_PI_F) / ENCODER_COUNTS_PER_REV;
      float elec_angle_rad = mech_angle_rad * (float)FOC_POLE_PAIRS;
      FOC_SensorData foc_data = {
        .ia = (float)ia_ma * 0.001f,
        .ib = (float)ib_ma * 0.001f,
        .ic = (float)ic_ma * 0.001f,
        .bus_v = (float)mv_bus * 0.001f,
        .electrical_angle = elec_angle_rad
      };
      FOC_UpdateSensors(&foc_data);

      UpdateMovingAverage(mv_ia, mv_ib, (uint32_t)mv_ic, mv_bus);

      if (filt_ready)
      {
        int32_t ia_ma_avg = ((int32_t)ia_mv_f - (int32_t)ia_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
        int32_t ib_ma_avg = ((int32_t)ib_mv_f - (int32_t)ib_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
        int32_t ic_ma_avg = ((int32_t)ic_mv_f - (int32_t)ic_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);

        uint32_t bus_v = (uint32_t)bus_mv_f / 1000U;
        uint32_t bus_mv_rem = (uint32_t)bus_mv_f % 1000U;

        // Convert floats to integers for printf
        int32_t deg_int = (int32_t)encoder_angle_deg;
        int32_t deg_frac = (int32_t)((encoder_angle_deg - (float)deg_int) * 10.0f);
        int32_t speed_int = (int32_t)encoder_speed;
        int32_t speed_frac = (int32_t)((encoder_speed - (float)speed_int) * 100.0f);
        if (speed_frac < 0) speed_frac = -speed_frac;

         printf("Ang:%5u(%ld.%1lddeg) Spd:%ld.%02ldrad/s | Iu:%5ldmA Iv:%5ldmA Iw:%5ldmA Bus:%3lu.%03luV\r\n",
           encoder_angle_raw, deg_int, deg_frac, speed_int, speed_frac,
           ia_ma_avg, ib_ma_avg, ic_ma_avg, bus_v, bus_mv_rem);
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
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
