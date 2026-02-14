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
#define BUS_VOLTAGE_DIVIDER_RATIO 12.93f  /* Adjust based on your resistor divider */
#define FOC_POLE_PAIRS 20U
#define ENCODER_COUNTS_PER_REV 32768.0f
#define TWO_PI_F 6.28318530718f

/* A1333 CS pin on PA15 (for SPI3) */
#define A1333_CS_PORT GPIOA
#define A1333_CS_PIN GPIO_PIN_15

/* Open-loop test parameters */
#define OL_VQ             2.0f       /* Voltage magnitude in volts (q-axis) */
#define OL_VD             0.0f       /* d-axis voltage (keep 0 for torque-producing) */
#define OL_SPEED_RAD_S    150.0f      /* Electrical rad/s for open-loop sweep */
#define OL_ALIGN_TIME_MS  1000U      /* Rotor alignment hold time in ms */
#define OL_ALIGN_VOLTAGE  2.0f       /* Voltage during alignment phase */
#define TIM1_PERIOD       5666U      /* TIM1 ARR value (30 kHz PWM) */
#define OL_LOOP_PERIOD_MS 5U         /* Fixed loop period for consistent timing */

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


static volatile uint32_t adc_dual_raw = 0;

/* A1333 Encoder Variables */
static A1333_Handle angle_sensor;
static uint16_t encoder_angle_raw = 0;
static float encoder_angle_deg = 0.0f;
static int16_t encoder_angle_prev = 0;
static uint32_t encoder_last_time = 0;

/* Open-loop state */
static float ol_electrical_angle = 0.0f;
static uint32_t ol_start_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Send(char* message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t ReadAdcInjectedCounts(ADC_HandleTypeDef *hadc)
{
  HAL_ADCEx_InjectedStop(hadc);  /* Stop any previous conversion */
  HAL_Delay(1);  /* Small delay */

  if (HAL_ADCEx_InjectedStart(hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedPollForConversion(hadc, 100) != HAL_OK)
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


static void SetPwmDuties(float du, float dv, float dw)
{
  if (du < 0.0f) { du = 0.0f; } if (du > 1.0f) { du = 1.0f; }
  if (dv < 0.0f) { dv = 0.0f; } if (dv > 1.0f) { dv = 1.0f; }
  if (dw < 0.0f) { dw = 0.0f; } if (dw > 1.0f) { dw = 1.0f; }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(du * (float)TIM1_PERIOD));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * (float)TIM1_PERIOD));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dw * (float)TIM1_PERIOD));
}

static void EnablePwmOutputs(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

__attribute__((unused)) static void DisablePwmOutputs(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void OpenLoopSVPWM(float vd, float vq, float elec_angle, float bus_v, float *du, float *dv, float *dw)
{
  float sin_t, cos_t;
  FOC_SinCos(elec_angle, &sin_t, &cos_t);

  float v_alpha = vd * cos_t - vq * sin_t;
  float v_beta  = vd * sin_t + vq * cos_t;

  float v_u = v_alpha;
  float v_v = -0.5f * v_alpha + 0.866025404f * v_beta;
  float v_w = -0.5f * v_alpha - 0.866025404f * v_beta;\

  /* SVPWM center-clamping */
  float v_max = v_u > v_v ? v_u : v_v; if (v_w > v_max) v_max = v_w;
  float v_min = v_u < v_v ? v_u : v_v; if (v_w < v_min) v_min = v_w;
  float v_offset = 0.5f * (v_max + v_min);
  v_u -= v_offset;
  v_v -= v_offset;
  v_w -= v_offset;

  float inv_bus = 1.0f / bus_v;
  *du = 0.5f + v_u * inv_bus;
  *dv = 0.5f + v_v * inv_bus;
  *dw = 0.5f + v_w * inv_bus;
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

  /* Enable UCC27302A gate driver (PC7, PC8, PC9) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  UART_Send("Hello, Mako Shortfin!\n");
  printf("Clock: %lu Hz\r\n", SystemCoreClock);
  printf("=== Open-Loop FOC Spin Test ===\r\n");
  printf("Vq=1.5V  Speed=10 rad/s  Poles=%u\r\n", FOC_POLE_PAIRS);

  FOC_Init();
  FOC_SetPolePairs(FOC_POLE_PAIRS);

  /* A1333 Encoder Init */
  A1333_Status a1333_status = A1333_Init(&angle_sensor, &hspi3, A1333_CS_PORT, A1333_CS_PIN);
  if (a1333_status != A1333_OK)
  {
    printf("A1333 init FAILED (%d)\r\n", a1333_status);
  }
  else
  {
    printf("A1333 init OK\r\n");
    A1333_ClearErrors(&angle_sensor);
    A1333_ClearWarnings(&angle_sensor);

    bool valid = false;
    for (int i = 0; i < 20 && !valid; i++)
    {
      A1333_IsAngleValid(&angle_sensor, &valid);
      HAL_Delay(10);
    }
    A1333_ReadAngle15(&angle_sensor, &encoder_angle_raw, &encoder_angle_deg);
    encoder_angle_prev = (int16_t)encoder_angle_raw;
    encoder_last_time = HAL_GetTick();
  }

  /* Start OPAMP, calibrate ADCs */
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  /* Wait for ADC voltage regulator to stabilize after calibration (20us typical) */
  printf("Waiting for ADC voltage regulator stabilization...\r\n");
  HAL_Delay(1);

  /* Start dual-mode ADC DMA with timer trigger */
  printf("Starting dual-mode ADC DMA...\r\n");
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&adc_dual_raw, 1) != HAL_OK) {
    printf("ERROR: MultiMode DMA start failed!\r\n");
    Error_Handler();
  }

  /* CRITICAL FIX: HAL doesn't set DMAEN bit - set it manually! */
  SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);
  printf("ADC DMA configured (DMAEN bit manually set).\r\n");

  /* Start TIM1 base + CH4 (ADC trigger) - AFTER ADCs are ready */
  printf("Starting TIM1 for PWM and ADC trigger...\r\n");
  if (HAL_TIM_Base_Start(&htim1) != HAL_OK) Error_Handler();
  if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
  printf("TIM1 started (TRGO2 now triggering ADCs)\r\n");

  /* Verify TIM1 CH4 configuration */
  uint32_t ccr4_value = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
  printf("TIM1: ARR=%lu, CCR4=%lu (should be ~%u)\r\n",
         htim1.Instance->ARR, ccr4_value, TIM1_PERIOD/2);

  /* Disable DMA interrupts (we'll just poll the data) */
  if (hadc1.DMA_Handle != NULL) {
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
  }

  /* Verify DMA configuration */
  if (hadc1.DMA_Handle != NULL) {
    printf("DMA1_Ch1: EN=%lu, CNDTR=%lu\r\n",
           (hadc1.DMA_Handle->Instance->CCR & DMA_CCR_EN) ? 1UL : 0UL,
           hadc1.DMA_Handle->Instance->CNDTR);
    printf("DMA CPAR=0x%08lX (peripheral addr)\r\n", hadc1.DMA_Handle->Instance->CPAR);
    printf("DMA CMAR=0x%08lX (memory addr, should be 0x%08lX)\r\n",
           hadc1.DMA_Handle->Instance->CMAR, (uint32_t)&adc_dual_raw);
    printf("ADC_COMMON CDR addr=0x%08lX\r\n", (uint32_t)&(ADC12_COMMON->CDR));
    printf("ADC1 CFGR=0x%08lX (DMAEN bit should be set)\r\n", hadc1.Instance->CFGR);
  } else {
    printf("ERROR: ADC1 DMA handle is NULL!\r\n");
  }

  printf("Waiting for ADC triggers...\r\n");
  HAL_Delay(100);

  /* Check if DMA is being updated by timer */
  uint32_t test1 = adc_dual_raw;
  HAL_Delay(10);
  uint32_t test2 = adc_dual_raw;
  HAL_Delay(10);
  uint32_t test3 = adc_dual_raw;

  printf("DMA test: #1=0x%08lX  #2=0x%08lX  #3=0x%08lX\r\n", test1, test2, test3);

  /* Check ADC status flags */
  uint32_t adc1_isr = hadc1.Instance->ISR;
  uint32_t adc2_isr = hadc2.Instance->ISR;
  printf("ADC1 ISR=0x%08lX (RDY=%lu EOC=%lu)\r\n", adc1_isr,
         (adc1_isr & ADC_ISR_ADRDY) ? 1UL : 0UL,
         (adc1_isr & ADC_ISR_EOC) ? 1UL : 0UL);
  printf("ADC2 ISR=0x%08lX (RDY=%lu EOC=%lu)\r\n", adc2_isr,
         (adc2_isr & ADC_ISR_ADRDY) ? 1UL : 0UL,
         (adc2_isr & ADC_ISR_EOC) ? 1UL : 0UL);

  if (test1 == 0 && test2 == 0 && test3 == 0) {
    printf("ERROR: ADC DMA still not updating!\r\n");
    printf("Continuing anyway - motor will spin but no current sense\r\n");
    ia_zero_mv = CURRENT_ZERO_MV;
    ib_zero_mv = CURRENT_ZERO_MV;
  } else if (test1 == test2 && test2 == test3) {
    printf("WARNING: ADC values not changing (stuck at 0x%08lX)\r\n", test1);
    uint16_t ia_raw = (uint16_t)(test1 & 0xFFFF);
    uint16_t ib_raw = (uint16_t)(test1 >> 16);
    ia_zero_mv = AdcCountsToMv(ia_raw);
    ib_zero_mv = AdcCountsToMv(ib_raw);
  } else {
    /* DMA working! Zero the currents */
    uint16_t ia_raw = (uint16_t)(adc_dual_raw & 0xFFFF);
    uint16_t ib_raw = (uint16_t)(adc_dual_raw >> 16);
    ia_zero_mv = AdcCountsToMv(ia_raw);
    ib_zero_mv = AdcCountsToMv(ib_raw);
    printf("SUCCESS: ADC DMA working! (values changing)\r\n");
  }

  printf("ADC zeroed: Ia=%lu mV  Ib=%lu mV\r\n", ia_zero_mv, ib_zero_mv);

  /* ---- Phase 1: Alignment ---- */
  printf("Aligning rotor to electrical angle 0...\r\n");
  SetPwmDuties(0.5f, 0.5f, 0.5f);
  EnablePwmOutputs();
  __HAL_TIM_MOE_ENABLE(&htim1);  /* Enable main output for TIM1 */

  /* Read bus voltage for SVPWM scaling */
  uint32_t raw_opamp = ReadAdcInjectedCounts(&hadc1);
  uint32_t mv_opamp = AdcCountsToMv(raw_opamp);
  float bus_v = ((float)mv_opamp / 1000.0f) * BUS_VOLTAGE_DIVIDER_RATIO;
  printf("Bus voltage: %ld.%01ld V (ADC raw=%lu, mV=%lu, divider=%.2f)\r\n",
         (int32_t)bus_v, (int32_t)((bus_v - (int32_t)bus_v) * 10.0f),
         raw_opamp, mv_opamp, BUS_VOLTAGE_DIVIDER_RATIO);

  {
    float du, dv, dw;
    OpenLoopSVPWM(0.0f, OL_ALIGN_VOLTAGE, 0.0f, bus_v, &du, &dv, &dw);
    SetPwmDuties(du, dv, dw);
  }
  HAL_Delay(OL_ALIGN_TIME_MS);
  printf("Alignment done. Starting open-loop spin.\r\n");

  /* ---- Phase 2: Open-loop spin ---- */
  ol_electrical_angle = 0.0f;
  ol_start_time = HAL_GetTick();

  uint32_t last_print_ms = HAL_GetTick();
  float du = 0.5f, dv = 0.5f, dw = 0.5f;  /* PWM duty cycles */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t loop_start = HAL_GetTick();

    /* Advance electrical angle with fixed time step */
    float dt_s = (float)OL_LOOP_PERIOD_MS * 0.001f;
    ol_electrical_angle += OL_SPEED_RAD_S * dt_s;
    if (ol_electrical_angle > TWO_PI_F) ol_electrical_angle -= TWO_PI_F;

    /* Compute SVPWM and drive motor */
    OpenLoopSVPWM(OL_VD, OL_VQ, ol_electrical_angle, bus_v, &du, &dv, &dw);
    SetPwmDuties(du, dv, dw);

    /* Read encoder (non-blocking) */
    A1333_ReadAngle15(&angle_sensor, &encoder_angle_raw, &encoder_angle_deg);

    /* Telemetry every 200ms */
    if ((loop_start - last_print_ms) >= 200U)
    {
      last_print_ms = loop_start;

      /* Read phase currents from DMA buffer */
      uint32_t raw = adc_dual_raw;
      uint16_t raw_ia = (uint16_t)(raw & 0xFFFF);
      uint16_t raw_ib = (uint16_t)(raw >> 16);
      uint32_t mv_ia = AdcCountsToMv(raw_ia);
      uint32_t mv_ib = AdcCountsToMv(raw_ib);
      int32_t ia_ma = ((int32_t)mv_ia - (int32_t)ia_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
      int32_t ib_ma = ((int32_t)mv_ib - (int32_t)ib_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
      int32_t ic_ma = -(ia_ma + ib_ma);

      int32_t el_deg = (int32_t)(ol_electrical_angle * 57.2957795f);
      int32_t enc_int = (int32_t)encoder_angle_deg;

      int32_t du_pct = (int32_t)(du * 100.0f);
      int32_t dv_pct = (int32_t)(dv * 100.0f);
      int32_t dw_pct = (int32_t)(dw * 100.0f);

      printf("eAng:%4ld  Enc:%4lddeg | Iu:%5ldmA(%4lumV) Iv:%5ldmA(%4lumV) Iw:%5ldmA | D:%ld/%ld/%ld%%\r\n",
             el_deg, enc_int, ia_ma, mv_ia, ib_ma, mv_ib, ic_ma, du_pct, dv_pct, dw_pct);
    }

    /* Fixed loop timing: wait until period elapsed */
    while ((HAL_GetTick() - loop_start) < OL_LOOP_PERIOD_MS) {
      /* Busy wait for consistent timing */
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
