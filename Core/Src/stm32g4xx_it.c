/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOC.h"
#include "tim.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A1333_CS_PORT       GPIOA
#define A1333_CS_PIN        GPIO_PIN_15
#define A1333_ANG15_CMD     0x3200U   /* Read ANG15 register (addr 0x32 << 8) */
#define A1333_NOP_CMD       0x0000U
#define FOC_POLE_PAIRS      20U
#define TWO_PI_F            6.28318530718f
#define TIM1_PERIOD         2125U
#define TIM1_HALF_PERIOD    (TIM1_PERIOD / 2U)
#define SPI_TIMEOUT_CYCLES  5100U     /* ~30us at 170MHz — SPI completes in <10us */
#define SPI_MAX_CONSECUTIVE_FAILS 5U  /* Disable FOC after N consecutive SPI failures */
#define ABSOLUTE_MAX_CURRENT_A 8.0f   /* Conservative: allow 8A peaks for PI settling */
#define DISABLE_OC_FOR_DEBUG 1        /* TEMP: disabled to observe steady-state behavior */
#define INVERT_CURRENT_POLARITY 1     /* Confirmed: windup test shows INVERT=1 is correct sign */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart1;
extern volatile uint32_t adc_dual_raw;
extern uint32_t ia_zero_mv;
extern uint32_t ib_zero_mv;
extern volatile float shared_electrical_angle;
extern volatile float shared_bus_voltage;
extern volatile uint8_t foc_isr_enabled;
extern volatile uint32_t foc_isr_count;
extern volatile uint32_t foc_isr_max_cycles;
extern float encoder_offset_rad;
extern volatile uint8_t foc_fault_code;
extern uint32_t adc_vref_mv;
extern volatile float shared_id_measured;
extern volatile float shared_iq_measured;
extern volatile float shared_vd;
extern volatile float shared_vq;
extern volatile float shared_mechanical_angle;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* Direct-register SPI3 16-bit transfer.
 * Cannot use HAL_SPI_TransmitReceive here because HAL_GetTick() is frozen
 * (SysTick is lower priority than this ISR). Uses loop counter for timeout
 * (DWT CYCCNT may not work without debugger connected). */
static inline uint16_t SPI3_Transfer16_Fast(uint16_t tx)
{
  volatile uint32_t timeout;

  /* Wait for TX buffer empty */
  timeout = SPI_TIMEOUT_CYCLES;
  while (!(SPI3->SR & SPI_SR_TXE))
  {
    if (--timeout == 0U) return 0xFFFF;
  }

  /* Write 16-bit data */
  *((__IO uint16_t *)&SPI3->DR) = tx;

  /* Wait for RX buffer not empty */
  timeout = SPI_TIMEOUT_CYCLES;
  while (!(SPI3->SR & SPI_SR_RXNE))
  {
    if (--timeout == 0U) return 0xFFFF;
  }

  uint16_t rx = *((__IO uint16_t *)&SPI3->DR);

  /* Wait until not busy */
  timeout = SPI_TIMEOUT_CYCLES;
  while (SPI3->SR & SPI_SR_BSY)
  {
    if (--timeout == 0U) return 0xFFFF;
  }

  return rx;
}

/* Read A1333 15-bit angle directly in ISR context.
 * Returns electrical angle in radians, or last good angle on SPI failure.
 * Disables FOC after SPI_MAX_CONSECUTIVE_FAILS consecutive timeouts. */
static float ReadEncoderInISR(void)
{
  static float last_good_angle = 0.0f;
  static uint8_t spi_fail_count = 0;

  /* Frame 1: Send ANG15 read command, response is stale (ignore) */
  A1333_CS_PORT->BRR = A1333_CS_PIN;        /* CS low */
  uint16_t rx1 = SPI3_Transfer16_Fast(A1333_ANG15_CMD);
  A1333_CS_PORT->BSRR = A1333_CS_PIN;       /* CS high */
  if (rx1 == 0xFFFF) goto spi_fail;

  /* CS-high idle delay >350ns for reliable A1333 framing.
   * ~15 loop iterations at 170 MHz ≈ 350ns */
  { volatile uint32_t d = 15U; while (d--); }

  /* Frame 2: Send NOP, read angle response */
  A1333_CS_PORT->BRR = A1333_CS_PIN;        /* CS low */
  uint16_t rx2 = SPI3_Transfer16_Fast(A1333_NOP_CMD);
  A1333_CS_PORT->BSRR = A1333_CS_PIN;       /* CS high */
  if (rx2 == 0xFFFF) goto spi_fail;

  spi_fail_count = 0;  /* Reset on success */

  /* Extract 15-bit angle and convert to electrical radians */
  uint16_t angle_raw = rx2 & 0x7FFF;
  float mech_rad = (float)angle_raw * (TWO_PI_F / 32768.0f);
  float elec = fmodf(mech_rad * (float)FOC_POLE_PAIRS, TWO_PI_F) - encoder_offset_rad;
  if (elec < 0.0f) elec += TWO_PI_F;

  shared_mechanical_angle = mech_rad;
  last_good_angle = elec;
  return elec;

spi_fail:
  spi_fail_count++;
  if (spi_fail_count >= SPI_MAX_CONSECUTIVE_FAILS)
  {
    /* Too many consecutive failures — enter safe state */
    foc_isr_enabled = 0;
    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;
  }
  return last_good_angle;
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void TIM1_UP_TIM16_IRQHandler(void)
{
  /* Clear update interrupt flag immediately */
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

  if (!foc_isr_enabled)
  {
    /* Safe state: 50% duty (zero voltage) */
    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;
    return;
  }

  /* Profiling: use SysTick->VAL as a free-running downcounter (no DWT dependency) */
  uint32_t cyc_start = SysTick->VAL;

  /* 1. Read phase currents from DMA buffer */
  uint32_t raw = adc_dual_raw;
  uint16_t raw_ia = (uint16_t)(raw & 0xFFFF);
  uint16_t raw_ib = (uint16_t)(raw >> 16);

  /* Convert ADC counts -> millivolts -> Amps (inlined for speed) */
  float ia_mv = (float)(raw_ia * adc_vref_mv) / 4095.0f;
  float ib_mv = (float)(raw_ib * adc_vref_mv) / 4095.0f;

#if INVERT_CURRENT_POLARITY
  float ia_raw = -((ia_mv - (float)ia_zero_mv) / 20.0f);
  float ib_raw = -((ib_mv - (float)ib_zero_mv) / 20.0f);
#else
  float ia_raw = (ia_mv - (float)ia_zero_mv) / 20.0f;
  float ib_raw = (ib_mv - (float)ib_zero_mv) / 20.0f;
#endif

  /* Low-pass filter on phase currents: fc ≈ 500Hz at 20kHz ISR rate
   * alpha = 2*pi*fc*dt / (1 + 2*pi*fc*dt) ≈ 0.14 */
  static float ia_filt = 0.0f, ib_filt = 0.0f;
  ia_filt += 0.15f * (ia_raw - ia_filt);
  ib_filt += 0.15f * (ib_raw - ib_filt);
  float ia = ia_filt;
  float ib = ib_filt;

  /* 1b. Overcurrent protection */
#if !DISABLE_OC_FOR_DEBUG
  if (fabsf(ia) > ABSOLUTE_MAX_CURRENT_A || fabsf(ib) > ABSOLUTE_MAX_CURRENT_A)
  {
    foc_isr_enabled = 0;
    foc_fault_code = 1;  /* overcurrent */

    /* Debug: Store raw values for main loop to print (no floats) */
    static volatile uint32_t debug_raw_ia, debug_raw_ib, debug_zero_ia, debug_zero_ib;
    debug_raw_ia = (uint32_t)ia_mv;
    debug_raw_ib = (uint32_t)ib_mv;
    debug_zero_ia = ia_zero_mv;
    debug_zero_ib = ib_zero_mv;
    (void)debug_raw_ia; (void)debug_raw_ib; (void)debug_zero_ia; (void)debug_zero_ib;

    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;
    return;
  }
#endif

  /* 2. Read encoder angle directly via SPI (no main-loop latency) */
  float elec_angle = ReadEncoderInISR();
  float bus_v = shared_bus_voltage;

  /* Write angle back so main loop can display it in telemetry */
  shared_electrical_angle = elec_angle;

  /* 3. Run FOC */
  FOC_SensorData sensors;
  sensors.ia = ia;
  sensors.ib = ib;
  sensors.ic = -(ia + ib);
  sensors.bus_v = bus_v;
  sensors.electrical_angle = elec_angle;

  FOC_Output output;
  FOC_UpdateSensors(&sensors);
  FOC_Run(&output);

  /* 3b. Export FOC internals for main-loop telemetry */
  shared_id_measured = output.id_measured;
  shared_iq_measured = output.iq_measured;
  shared_vd = output.vd;
  shared_vq = output.vq;

  /* 4. Apply PWM via direct register writes (clamped to [0, TIM1_PERIOD]) */
  float ccr_u = output.duty_u * (float)TIM1_PERIOD;
  float ccr_v = output.duty_v * (float)TIM1_PERIOD;
  float ccr_w = output.duty_w * (float)TIM1_PERIOD;
  if (ccr_u < 0.0f) ccr_u = 0.0f; else if (ccr_u > (float)TIM1_PERIOD) ccr_u = (float)TIM1_PERIOD;
  if (ccr_v < 0.0f) ccr_v = 0.0f; else if (ccr_v > (float)TIM1_PERIOD) ccr_v = (float)TIM1_PERIOD;
  if (ccr_w < 0.0f) ccr_w = 0.0f; else if (ccr_w > (float)TIM1_PERIOD) ccr_w = (float)TIM1_PERIOD;
  TIM1->CCR1 = (uint32_t)ccr_u;
  TIM1->CCR2 = (uint32_t)ccr_v;
  TIM1->CCR3 = (uint32_t)ccr_w;

  /* 5. Profiling (SysTick counts DOWN, so start > end for elapsed) */
  uint32_t cyc_end = SysTick->VAL;
  uint32_t cyc_elapsed = (cyc_start >= cyc_end)
    ? (cyc_start - cyc_end)
    : (cyc_start + (SysTick->LOAD + 1U) - cyc_end);
  foc_isr_count++;
  if (cyc_elapsed > foc_isr_max_cycles)
  {
    foc_isr_max_cycles = cyc_elapsed;
  }
}
/* USER CODE END 1 */
