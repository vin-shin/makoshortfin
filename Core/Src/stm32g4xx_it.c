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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
    TIM1->CCR1 = 2833U;
    TIM1->CCR2 = 2833U;
    TIM1->CCR3 = 2833U;
    return;
  }

  /* Start cycle counter for profiling */
  uint32_t cyc_start = DWT->CYCCNT;

  /* 1. Read phase currents from DMA buffer */
  uint32_t raw = adc_dual_raw;
  uint16_t raw_ia = (uint16_t)(raw & 0xFFFF);
  uint16_t raw_ib = (uint16_t)(raw >> 16);

  /* Convert ADC counts -> millivolts -> Amps (inlined for speed) */
  float ia_mv = (float)(raw_ia * 3300U) / 4095.0f;
  float ib_mv = (float)(raw_ib * 3300U) / 4095.0f;
  float ia = (ia_mv - (float)ia_zero_mv) / 20.0f;  /* 20 mV/A sensitivity */
  float ib = (ib_mv - (float)ib_zero_mv) / 20.0f;

  /* 2. Read shared data (single aligned float reads are atomic on CM4) */
  float elec_angle = shared_electrical_angle;
  float bus_v = shared_bus_voltage;

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

  /* 4. Apply PWM via direct register writes (no HAL overhead) */
  TIM1->CCR1 = (uint32_t)(output.duty_u * 5666.0f);
  TIM1->CCR2 = (uint32_t)(output.duty_v * 5666.0f);
  TIM1->CCR3 = (uint32_t)(output.duty_w * 5666.0f);

  /* 5. Profiling */
  uint32_t cyc_elapsed = DWT->CYCCNT - cyc_start;
  foc_isr_count++;
  if (cyc_elapsed > foc_isr_max_cycles)
  {
    foc_isr_max_cycles = cyc_elapsed;
  }
}
/* USER CODE END 1 */
