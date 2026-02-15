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

/* FOC control parameters */
#define FOC_IQ_REF_A      0.0f       /* Target q-axis current (torque) in Amps */
#define FOC_ID_REF_A      0.0f       /* Target d-axis current (flux weakening) */
#define FOC_KP_D          0.1f       /* D-axis PI controller Kp (tuned for 15kHz) */
#define FOC_KI_D          50.0f      /* D-axis PI controller Ki */
#define FOC_KP_Q          0.1f       /* Q-axis PI controller Kp (tuned for 15kHz) */
#define FOC_KI_Q          50.0f      /* Q-axis PI controller Ki */
#define TIM1_PERIOD       5666U      /* TIM1 ARR value (30 kHz PWM) */
#define FOC_ISR_FREQ_HZ   15000U     /* FOC ISR rate (TIM1 30kHz / RCR+1) */
#define FOC_ISR_DT_SECONDS (1.0f / (float)FOC_ISR_FREQ_HZ)
#define FOC_ALIGN_TIME_MS 500U       /* Rotor alignment time in ms */
#define FOC_ALIGN_VOLTAGE 1.5f       /* Alignment voltage */
#define BUS_V_UPDATE_MS   100U       /* Bus voltage re-read interval */
#define UART_TX_BUF_SIZE  256U       /* Non-blocking UART transmit buffer */

/* Safety limits */
#define MAX_BUS_VOLTAGE   30.0f     /* Overvoltage shutdown threshold (V) */
#define MIN_BUS_VOLTAGE   8.0f      /* Undervoltage shutdown threshold (V) */

/* UART kill switch */
#define UART_RX_BUF_SIZE  1U

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
/* ISR-visible variables (non-static, accessed by TIM1 Update ISR) */
uint32_t ia_zero_mv = CURRENT_ZERO_MV;
uint32_t ib_zero_mv = CURRENT_ZERO_MV;
volatile uint32_t adc_dual_raw = 0;

/* Shared ISR <-> main loop state */
volatile float shared_electrical_angle = 0.0f;
volatile float shared_bus_voltage = 0.0f;
volatile uint8_t foc_isr_enabled = 0;
volatile uint32_t foc_isr_count = 0;
volatile uint32_t foc_isr_max_cycles = 0;

/* A1333 Encoder */
static A1333_Handle angle_sensor;
float encoder_offset_rad = 0.0f;  /* Non-static: ISR reads this */

/* Fault reporting (0=no fault, 1=overcurrent, 2=overvoltage, 3=undervoltage, 4=user kill) */
volatile uint8_t foc_fault_code = 0;

/* ISR -> main telemetry: measured dq currents and PI voltage outputs */
volatile float shared_id_measured = 0.0f;
volatile float shared_iq_measured = 0.0f;
volatile float shared_vd = 0.0f;
volatile float shared_vq = 0.0f;

/* Non-blocking UART telemetry */
static char uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t uart_tx_busy = 0;

/* UART receive for kill switch */
static uint8_t uart_rx_byte = 0;

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

static float ReadBusVoltageBlocking(void)
{
  uint32_t raw = ReadAdcInjectedCounts(&hadc1);
  uint32_t mv = AdcCountsToMv(raw);
  return ((float)mv / 1000.0f) * BUS_VOLTAGE_DIVIDER_RATIO;
}

static void AlignRotorSVPWM(float vd, float vq, float elec_angle, float bus_v)
{
  float sin_t, cos_t;
  FOC_SinCos(elec_angle, &sin_t, &cos_t);

  float v_alpha = vd * cos_t - vq * sin_t;
  float v_beta  = vd * sin_t + vq * cos_t;

  float v_u = v_alpha;
  float v_v = -0.5f * v_alpha + 0.866025404f * v_beta;
  float v_w = -0.5f * v_alpha - 0.866025404f * v_beta;

  float v_max = v_u > v_v ? v_u : v_v; if (v_w > v_max) v_max = v_w;
  float v_min = v_u < v_v ? v_u : v_v; if (v_w < v_min) v_min = v_w;
  float v_offset = 0.5f * (v_max + v_min);
  v_u -= v_offset;
  v_v -= v_offset;
  v_w -= v_offset;

  float inv_bus = 1.0f / bus_v;
  SetPwmDuties(0.5f + v_u * inv_bus, 0.5f + v_v * inv_bus, 0.5f + v_w * inv_bus);
}

static float CalibrateEncoderOffset(float bus_v)
{
  /* Phase 1: Apply current at electrical angle 0 to lock rotor */
  printf("Aligning rotor to electrical zero...\r\n");
  AlignRotorSVPWM(0.0f, FOC_ALIGN_VOLTAGE, 0.0f, bus_v);
  HAL_Delay(FOC_ALIGN_TIME_MS);

  /* Phase 2: Read encoder angle at the locked position */
  float sum_deg = 0.0f;
  uint16_t raw;
  float deg;
  for (int i = 0; i < 16; i++)
  {
    A1333_ReadAngle15(&angle_sensor, &raw, &deg);
    sum_deg += deg;
    HAL_Delay(5);
  }
  float mech_at_zero_deg = sum_deg / 16.0f;
  float mech_at_zero_rad = mech_at_zero_deg * (TWO_PI_F / 360.0f);

  /* The electrical angle at alignment is 0, so:
     electrical_angle = (mechanical * pole_pairs) - offset = 0
     offset = mechanical * pole_pairs (wrapped to [0, 2*pi]) */
  float offset = mech_at_zero_rad * (float)FOC_POLE_PAIRS;
  while (offset > TWO_PI_F) offset -= TWO_PI_F;
  while (offset < 0.0f) offset += TWO_PI_F;

  printf("Encoder offset: mech=%.1f deg  offset=%.3f rad\r\n", mech_at_zero_deg, offset);

  /* Ramp down alignment voltage */
  for (int i = 10; i >= 0; i--)
  {
    float v = FOC_ALIGN_VOLTAGE * ((float)i / 10.0f);
    AlignRotorSVPWM(0.0f, v, 0.0f, bus_v);
    HAL_Delay(20);
  }
  SetPwmDuties(0.5f, 0.5f, 0.5f);

  return offset;
}

static void UART_SendNonBlocking(const char *buf, uint16_t len)
{
  if (uart_tx_busy) return;  /* Drop if previous transfer not done */
  uart_tx_busy = 1;
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)buf, len);
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

  /* Configure NVIC priorities */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Enable UCC27302A gate driver (PC7, PC8, PC9) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

  UART_Send("Hello, Mako Shortfin!\n");
  printf("Clock: %lu Hz\r\n", SystemCoreClock);
  printf("=== ISR-Based FOC Control @ %u Hz ===\r\n", FOC_ISR_FREQ_HZ);
  printf("Iq_ref=%.1fA  Id_ref=%.1fA  Poles=%u\r\n", FOC_IQ_REF_A, FOC_ID_REF_A, FOC_POLE_PAIRS);
  printf("Current PI: Kp=%.2f Ki=%.1f\r\n", FOC_KP_Q, FOC_KI_Q);

  FOC_Init();
  FOC_SetPolePairs(FOC_POLE_PAIRS);
  FOC_SetControlPeriod(FOC_ISR_DT_SECONDS);
  FOC_SetCurrentGains(FOC_KP_D, FOC_KI_D, FOC_KP_Q, FOC_KI_Q);
  FOC_SetCurrentRefs(FOC_ID_REF_A, FOC_IQ_REF_A);

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
    uint16_t init_raw;
    float init_deg;
    A1333_ReadAngle15(&angle_sensor, &init_raw, &init_deg);
  }

  /* Start OPAMP, calibrate ADCs */
  if (HAL_OPAMP_Start(&hopamp3) != HAL_OK) Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  /* Start dual-mode ADC DMA with timer trigger */
  HAL_Delay(1);
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&adc_dual_raw, 1) != HAL_OK) {
    printf("ERROR: ADC DMA start failed!\r\n");
    Error_Handler();
  }

  /* CRITICAL: HAL doesn't set DMAEN bit - set it manually */
  SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN);

  /* Start TIM1 base + CH4 (ADC trigger) */
  if (HAL_TIM_Base_Start(&htim1) != HAL_OK) Error_Handler();
  if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  /* Disable DMA interrupts (polling mode for ADC data) */
  if (hadc1.DMA_Handle != NULL) {
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
  }

  /* Zero the current sensors */
  HAL_Delay(100);
  uint16_t ia_raw = (uint16_t)(adc_dual_raw & 0xFFFF);
  uint16_t ib_raw = (uint16_t)(adc_dual_raw >> 16);
  ia_zero_mv = AdcCountsToMv(ia_raw);
  ib_zero_mv = AdcCountsToMv(ib_raw);
  printf("ADC zeroed: Ia=%lu mV, Ib=%lu mV\r\n", ia_zero_mv, ib_zero_mv);

  /* ISR handles current offset subtraction inline, no need for FOC_SetCurrentOffsets */

  /* Read bus voltage (blocking OK during init, before ISR starts) */
  float bus_v = ReadBusVoltageBlocking();
  shared_bus_voltage = bus_v;
  printf("Bus: %ld.%01ld V\r\n",
         (int32_t)bus_v, (int32_t)((bus_v - (int32_t)bus_v) * 10.0f));

  /* Enable PWM outputs */
  SetPwmDuties(0.5f, 0.5f, 0.5f);
  EnablePwmOutputs();
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* Calibrate encoder offset by aligning rotor to electrical zero */
  encoder_offset_rad = CalibrateEncoderOffset(bus_v);

  /* Reset PI integrators after alignment (keeps all config intact) */
  FOC_ResetIntegrators();

  /* Enable DWT cycle counter for ISR profiling */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Set initial shared state */
  shared_electrical_angle = 0.0f;

  /* Enable TIM1 Update interrupt (FOC ISR at 15 kHz) */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);  /* Highest priority */
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

  /* Start FOC processing in ISR */
  foc_isr_enabled = 1;
  printf("FOC ISR started @ %u Hz\r\n", FOC_ISR_FREQ_HZ);

  /* Start IWDG watchdog (~1s timeout) */
  IWDG->KR  = 0x5555U;   /* Unlock registers */
  IWDG->PR  = 4U;         /* Prescaler /64 -> 40kHz/64 = 625Hz */
  IWDG->RLR = 625U;       /* Reload -> ~1s timeout */
  IWDG->KR  = 0xCCCCU;    /* Start watchdog */
  printf("IWDG watchdog started (~1s timeout)\r\n");

  /* Start UART receive interrupt for kill switch (send 'k' to disable FOC) */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, UART_RX_BUF_SIZE);
  printf("Safety: OC=10A, Bus=[%.0f-%.0fV], UART 'k'=kill\r\n\r\n",
         MIN_BUS_VOLTAGE, MAX_BUS_VOLTAGE);

  uint32_t last_print_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* --- Task 1: Non-blocking bus voltage state machine --- */
    {
      static enum { BV_IDLE, BV_WAIT } bv_state = BV_IDLE;
      static uint32_t bv_last_ms = 0;

      if (bv_state == BV_IDLE && (now - bv_last_ms) >= BUS_V_UPDATE_MS)
      {
        /* Only start injected when TIM1 counter is far from ADC trigger (CH4=2833) */
        uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim1);
        if (cnt < 1000U || cnt > 4500U)
        {
          HAL_ADCEx_InjectedStart(&hadc1);
          bv_state = BV_WAIT;
        }
      }
      if (bv_state == BV_WAIT)
      {
        if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS))
        {
          uint32_t raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
          HAL_ADCEx_InjectedStop(&hadc1);
          __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS | ADC_FLAG_JEOC);

          float v = ((float)AdcCountsToMv(raw) / 1000.0f) * BUS_VOLTAGE_DIVIDER_RATIO;
          __disable_irq();
          shared_bus_voltage = v;
          __enable_irq();

          /* Bus voltage window check */
          if (foc_isr_enabled && (v > MAX_BUS_VOLTAGE || v < MIN_BUS_VOLTAGE))
          {
            foc_isr_enabled = 0;
            foc_fault_code = (v > MAX_BUS_VOLTAGE) ? 2 : 3;
          }

          bv_state = BV_IDLE;
          bv_last_ms = now;
        }
      }
    }

    /* --- Task 2: Refresh watchdog --- */
    IWDG->KR = 0xAAAAU;

    /* --- Task 3: Non-blocking telemetry every 20ms --- */
    if ((now - last_print_ms) >= 20U)
    {
      last_print_ms = now;

      /* Read ISR profiling data */
      uint32_t isr_cnt = foc_isr_count;
      uint32_t max_cyc = foc_isr_max_cycles;
      float max_us = (float)max_cyc / 170.0f;  /* 170 MHz -> cycles to us */

      int32_t el_deg = (int32_t)(shared_electrical_angle * 57.2957795f);
      int32_t bus_mv = (int32_t)(shared_bus_voltage * 1000.0f);
      float id_m = shared_id_measured;
      float iq_m = shared_iq_measured;
      float vd_out = shared_vd;
      float vq_out = shared_vq;

      int len;
      if (foc_fault_code)
      {
        static const char *fault_names[] = {
          "none", "OVERCURRENT", "OVERVOLTAGE", "UNDERVOLTAGE", "USER_KILL"
        };
        const char *fname = (foc_fault_code <= 4) ? fault_names[foc_fault_code] : "UNKNOWN";
        len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
               "FAULT:%s Bus:%ld.%01ldV id:%.2f iq:%.2f\r\n",
               fname,
               bus_mv / 1000, (bus_mv % 1000) / 100,
               (double)id_m, (double)iq_m);
      }
      else
      {
        len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
               "θe:%3ld° Bus:%ld.%01ldV id:%.2f iq:%.2f vd:%.1f vq:%.1f %luHz %.1fus\r\n",
               el_deg,
               bus_mv / 1000, (bus_mv % 1000) / 100,
               (double)id_m, (double)iq_m,
               (double)vd_out, (double)vq_out,
               isr_cnt * 50UL,
               (double)max_us);
      }
      if (len > 0) UART_SendNonBlocking(uart_tx_buf, (uint16_t)len);

      foc_isr_count = 0;       /* Reset count for next telemetry period */
      foc_isr_max_cycles = 0;  /* Reset peak tracker */
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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_tx_busy = 0;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (uart_rx_byte == 'k' || uart_rx_byte == 'K')
    {
      foc_isr_enabled = 0;
      foc_fault_code = 4;  /* user kill */
    }
    /* Re-arm receive for next byte */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, UART_RX_BUF_SIZE);
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
