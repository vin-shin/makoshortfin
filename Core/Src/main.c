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
#include "a1333.h"
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
#define AVG_WINDOW 64U

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

static uint32_t ia_mv_buf[AVG_WINDOW] = {0};
static uint32_t ib_mv_buf[AVG_WINDOW] = {0};
static uint32_t ic_mv_buf[AVG_WINDOW] = {0};
static uint32_t bus_mv_buf[AVG_WINDOW] = {0};
static uint32_t avg_index = 0;
static uint32_t avg_count = 0;
static uint32_t ia_mv_sum = 0;
static uint32_t ib_mv_sum = 0;
static uint32_t ic_mv_sum = 0;
static uint32_t bus_mv_sum = 0;

/* A1333 Encoder Variables */
static A1333_Handle angle_sensor;
static uint16_t encoder_angle_raw = 0;
static float encoder_angle_deg = 0.0f;
static int16_t encoder_angle_prev = 0;
static float encoder_speed = 0.0f;  // rad/s
static uint32_t encoder_last_time = 0;

/* Motor Control Variables */
// *** ADJUST THESE SETTINGS FOR YOUR MOTOR ***
#define MOTOR_POLE_PAIRS 20          // Number of pole pairs (confirmed: 20)
static uint32_t motor_duty_percent = 20;  // Starting duty cycle (5-50%)
static int8_t motor_direction = 1;        // 1 for forward, -1 for reverse

// Encoder offset and direction
// Set force_encoder_direction = 1 to manually override auto-detection
static uint8_t force_encoder_direction = 1;  // 1 = override, 0 = use auto-detection
static int8_t encoder_direction_override = -1;  // Manual: 1 = normal, -1 = inverted
static int16_t encoder_offset = 0;
static int8_t encoder_direction = 1;     // Auto-detected value

// Test mode
static uint8_t open_loop_test = 0;       // 1 = open-loop test, 0 = closed-loop with encoder
static uint8_t skip_calibration = 0;     // 0 = run calibration (recommended), 1 = skip

// Internal variables
static uint32_t pwm_period = 0;
static uint8_t motor_enabled = 0;
static uint8_t last_sector = 0xFF;       // Track sector changes
static uint16_t open_loop_angle = 0;     // Simulated angle for open-loop

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

static uint32_t ReadAdcCounts(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start(hadc) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t value = HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  return value;
}

static uint32_t AdcCountsToMv(uint32_t counts)
{
  return (counts * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static void AutoZeroCurrents(void)
{
  const uint32_t samples = 256U;
  uint64_t sum_ia = 0;
  uint64_t sum_ib = 0;
  uint64_t sum_ic = 0;

  for (uint32_t i = 0; i < samples; ++i)
  {
    sum_ia += AdcCountsToMv(ReadAdcCounts(&hadc1, ADC_CHANNEL_1));
    sum_ib += AdcCountsToMv(ReadAdcCounts(&hadc2, ADC_CHANNEL_3));
    sum_ic += AdcCountsToMv(ReadAdcCounts(&hadc2, ADC_CHANNEL_4));
  }

  ia_zero_mv = (uint32_t)(sum_ia / samples);
  ib_zero_mv = (uint32_t)(sum_ib / samples);
  ic_zero_mv = (uint32_t)(sum_ic / samples);
}

static void UpdateMovingAverage(uint32_t ia_mv, uint32_t ib_mv, uint32_t ic_mv, uint32_t bus_mv)
{
  if (avg_count < AVG_WINDOW)
  {
    avg_count++;
  }
  else
  {
    ia_mv_sum -= ia_mv_buf[avg_index];
    ib_mv_sum -= ib_mv_buf[avg_index];
    ic_mv_sum -= ic_mv_buf[avg_index];
    bus_mv_sum -= bus_mv_buf[avg_index];
  }

  ia_mv_buf[avg_index] = ia_mv;
  ib_mv_buf[avg_index] = ib_mv;
  ic_mv_buf[avg_index] = ic_mv;
  bus_mv_buf[avg_index] = bus_mv;
  ia_mv_sum += ia_mv;
  ib_mv_sum += ib_mv;
  ic_mv_sum += ic_mv;
  bus_mv_sum += bus_mv;

  avg_index++;
  if (avg_index >= AVG_WINDOW)
  {
    avg_index = 0;
  }
}

/**
 * @brief Robust encoder calibration - finds offset algorithmically
 * @return Calculated encoder offset
 * 
 * Algorithm:
 * 1. Force motor to electrical angle 0 by energizing V+ W- (sector 0 commutation)
 * 2. Wait for rotor to physically align with the magnetic field
 * 3. Read encoder position - this is the mechanical angle when electrical = 0
 * 4. Calculate offset so that: (measured_angle + offset) * pole_pairs ≡ 0 (mod 32768)
 * 5. Simplest solution: offset = -measured_angle
 */
static int16_t CalibrateEncoderOffset(void)
{
  printf("\r\n=== Algorithmic Encoder Calibration ===\r\n");
  printf("Step 1: Forcing motor to electrical angle 0 (V+ W-)...\r\n");
  
  // Force specific commutation: V+ W- (sector 0, electrical angle ~0-60°)
  // Use strong torque to overcome any friction
  uint32_t align_duty = (pwm_period * 40) / 100;  // 40% duty
  
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, align_duty);      // V = HIGH (100% duty)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);               // W = LOW (0% duty)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);    // U = FLOAT (50% - no current)
  
  printf("Step 2: Waiting for rotor to settle (2.5s)...\r\n");
  HAL_Delay(2500);  // Generous settling time
  
  printf("Step 3: Reading encoder position...\r\n");
  
  // Read encoder multiple times and average for accuracy
  uint32_t angle_sum = 0;
  const uint8_t samples = 30;
  
  for (uint8_t i = 0; i < samples; i++)
  {
    uint16_t raw_angle;
    float deg;
    if (A1333_ReadAngle15(&angle_sensor, &raw_angle, &deg) == A1333_OK)
    {
      angle_sum += raw_angle;
    }
    HAL_Delay(10);
  }
  
  uint16_t measured_angle = (uint16_t)(angle_sum / samples);
  
  printf("Step 4: Calculating offset...\r\n");
  
  // The rotor is now physically aligned to electrical angle 0
  // We want: (encoder_reading + offset) * pole_pairs ≡ 0 (mod 32768)
  // Solution: offset = -encoder_reading
  // This makes adjusted_angle = 0, so electrical_angle = 0
  int16_t calculated_offset = -(int16_t)measured_angle;
  
  // Verify the calculation
  uint16_t test_adjusted = (measured_angle + calculated_offset) & 0x7FFF;
  uint32_t test_electrical = ((uint32_t)test_adjusted * MOTOR_POLE_PAIRS) % 32768;
  
  // Turn off alignment - all phases floating
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_period/2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);
  
  printf("\\r\\nCalibration Results:\\r\\n");
  printf("  Measured mechanical angle: %u (%.1f° mech)\\r\\n", 
         measured_angle, (float)measured_angle * 360.0f / 32768.0f);
  printf("  Calculated offset: %d\\r\\n", calculated_offset);
  printf("  Verification - Electrical angle: %lu (should be ~0)\\r\\n", test_electrical);
  
  // Step 5: Detect encoder direction by forcing next sector
  printf("\\r\\nStep 5: Detecting encoder direction...\\r\\n");
  printf("  Forcing next sector (V+ U-) for 1 second...\\r\\n");
  
  // Force sector 1: V+ U-
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, align_duty);      // V = HIGH
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);    // W = FLOAT
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);               // U = LOW
  
  HAL_Delay(1000);  // Let motor settle
  
  // Read encoder in new position
  uint32_t angle_sum2 = 0;
  for (uint8_t i = 0; i < samples; i++)
  {
    uint16_t raw_angle;
    float deg;
    if (A1333_ReadAngle15(&angle_sensor, &raw_angle, &deg) == A1333_OK)
    {
      angle_sum2 += raw_angle;
    }
    HAL_Delay(10);
  }
  uint16_t measured_angle2 = (uint16_t)(angle_sum2 / samples);
  
  // Calculate delta (accounting for wraparound)
  int32_t delta = (int32_t)measured_angle2 - (int32_t)measured_angle;
  if (delta > 16384) delta -= 32768;
  else if (delta < -16384) delta += 32768;
  
  // Detect direction: if motor advanced electrically forward and encoder increased, direction is normal
  // Sector 0->1 is forward electrical rotation
  if (delta > 0)
  {
    encoder_direction = 1;  // Encoder increases with forward electrical rotation
    printf("  Encoder direction: NORMAL (encoder increases with forward rotation)\\r\\n");
  }
  else if (delta < 0)
  {
    encoder_direction = -1;  // Encoder decreases with forward electrical rotation
    printf("  Encoder direction: INVERTED (encoder decreases with forward rotation)\\r\\n");
  }
  else
  {
    encoder_direction = 1;  // Default to normal if no change detected
    printf("  Encoder direction: UNKNOWN (no movement detected, assuming normal)\\r\\n");
  }
  
  // Turn off alignment - all phases floating
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_period/2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);
  
  printf("  Motor released.\\r\\n");
  
  return calculated_offset;
}

/**
 * @brief 6-step commutation control
 * @param angle_raw: Encoder angle (0-32767)
 * @param duty: PWM duty value (0 to pwm_period)
 * 
 * Phase mapping:
 * - Phase U = TIM1_CH3 (PA10/PB15)
 * - Phase V = TIM1_CH1 (PA8/PA11)
 * - Phase W = TIM1_CH2 (PA9/PA12)
 */
static void Motor_Commutate(uint16_t angle_raw, uint32_t duty)
{
  // Determine which encoder direction to use
  int8_t active_encoder_dir = force_encoder_direction ? encoder_direction_override : encoder_direction;
  
  // Apply encoder direction (invert if encoder direction is opposite to motor)
  uint16_t corrected_angle = angle_raw;
  if (active_encoder_dir < 0)
  {
    corrected_angle = 32768 - angle_raw;  // Invert encoder direction
  }
  
  // Apply encoder offset and calculate electrical angle
  uint16_t adjusted_angle = (corrected_angle + encoder_offset) & 0x7FFF;
  uint32_t elec_angle = ((uint32_t)adjusted_angle * MOTOR_POLE_PAIRS) % 32768;
  
  // Determine sector (0-5) based on electrical angle
  // Each sector is 60 degrees = 32768/6 = 5461.33 counts
  uint8_t sector = (uint8_t)((elec_angle * 6) / 32768);
  
  // Bounds check
  if (sector > 5) sector = 5;
  
  // Apply motor direction
  if (motor_direction < 0)
  {
    sector = (6 - sector) % 6;
  }
  
  // Debug output when sector changes (helps diagnose commutation issues)
  static uint32_t last_debug_print = 0;
  uint32_t now = HAL_GetTick();
  if (sector != last_sector && (now - last_debug_print) > 100)
  {
    last_sector = sector;
    last_debug_print = now;
    printf("Sec:%u Elec:%lu Mech:%u Raw:%u Off:%d\r\n", 
           sector, elec_angle, adjusted_angle, angle_raw, encoder_offset);
  }
  
  uint32_t duty_on = duty;
  uint32_t duty_off = 0;
  
  // 6-step commutation table
  // Each sector: one phase HIGH, one phase LOW, one phase FLOATING (PWM at 50%)
  switch (sector)
  {
    case 0:  // V+ W-  (U floating)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_on);   // V = CH1 = HIGH
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_off);  // W = CH2 = LOW
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);  // U = CH3 = FLOAT
      break;
      
    case 1:  // V+ U-  (W floating)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_on);   // V = CH1 = HIGH
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);  // W = CH2 = FLOAT
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_off);  // U = CH3 = LOW
      break;
      
    case 2:  // W+ U-  (V floating)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_period/2);  // V = CH1 = FLOAT
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_on);   // W = CH2 = HIGH
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_off);  // U = CH3 = LOW
      break;
      
    case 3:  // W+ V-  (U floating)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_off);  // V = CH1 = LOW
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_on);   // W = CH2 = HIGH
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);  // U = CH3 = FLOAT
      break;
      
    case 4:  // U+ V-  (W floating)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_off);  // V = CH1 = LOW
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);  // W = CH2 = FLOAT
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_on);   // U = CH3 = HIGH
      break;
      
    case 5:  // U+ W-  (V floating)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_period/2);  // V = CH1 = FLOAT
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_off);  // W = CH2 = LOW
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_on);   // U = CH3 = HIGH
      break;
  }
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
  
  A1333_Status a1333_status = A1333_Init(&angle_sensor, &hspi3, A1333_CS_PORT, A1333_CS_PIN);
  if (a1333_status != A1333_OK)
  {
    printf("A1333 init failed! Status: %d\r\n", a1333_status);
    printf("Check: wiring, power, SPI config (Mode 3, 16-bit)\r\n");
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

    uint32_t arr = 0;
    uint32_t last_print_ms = 0;
    uint32_t last_encoder_ms = 0;

    /* TIM1 runs from APB2 (170 MHz). Prescale to 170 MHz / 170 = 1 MHz, then set 30 kHz PWM. */
    __HAL_TIM_SET_PRESCALER(&htim1, 169);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 32);
    arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    pwm_period = arr + 1;  // Store for motor control
    
    /* Initialize all phases to 50% (floating) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);

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

    HAL_Delay(500);
    AutoZeroCurrents();

    /* Enable gate drivers on PC7, PC8, PC9 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
    printf("Gate drivers enabled (PC7, PC8, PC9)\r\n");

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    
    /* Run encoder calibration routine */
    if (open_loop_test)
    {
      printf("\r\n=== OPEN-LOOP TEST MODE ===\r\n");
      printf("Motor will spin slowly without encoder feedback\r\n");
      printf("This tests basic commutation and wiring\r\n");
    }
    else if (skip_calibration)
    {
      printf("\r\n=== MANUAL OFFSET MODE ===\r\n");
      printf("Using manually set encoder offset: %d\r\n", encoder_offset);
      printf("(Calibration skipped)\r\n");
    }
    else
    {
      printf("\r\n=== AUTOMATIC CALIBRATION ===\r\n");
      printf("For custom encoders, this algorithmically finds the offset\r\n");
      printf("by forcing a known electrical position and measuring encoder.\r\n");
      encoder_offset = CalibrateEncoderOffset();
    }
    
    printf("\r\n=== Motor Control Ready ===\r\n");
    printf("Final encoder offset: %d\r\n", encoder_offset);
    if (force_encoder_direction)
    {
      printf("Encoder direction: %s (MANUAL OVERRIDE)\r\n", 
             encoder_direction_override > 0 ? "NORMAL" : "INVERTED");
    }
    else
    {
      printf("Encoder direction: %s (auto-detected)\r\n", 
             encoder_direction > 0 ? "NORMAL" : "INVERTED");
    }
    printf("Auto-starting motor in 2 seconds...\r\n");
    HAL_Delay(2000);
    motor_enabled = 1;
    printf("Motor ENABLED at %lu%% duty, motor direction: %s\r\n\r\n",
           motor_duty_percent, motor_direction > 0 ? "FWD" : "REV");

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
    
    /* Motor commutation control - run at high frequency */
    if (motor_enabled)
    {
      // Calculate duty cycle from percentage
      uint32_t duty_value = (pwm_period * motor_duty_percent) / 100;
      if (duty_value > pwm_period) duty_value = pwm_period;
      
      uint16_t angle_to_use;
      
      if (open_loop_test)
      {
        // Open-loop test: smoothly increment angle for continuous rotation
        open_loop_angle += 8;  // Small increment every loop = smooth rotation
        if (open_loop_angle >= 32768) open_loop_angle -= 32768;
        angle_to_use = open_loop_angle;
      }
      else
      {
        // Closed-loop: use encoder
        angle_to_use = encoder_angle_raw;
      }
      
      // Apply commutation
      if (angle_to_use != 0 || open_loop_test)
      {
        Motor_Commutate(angle_to_use, duty_value);
      }
    }
    else if (!motor_enabled)
    {
      // Motor disabled - all phases floating (50%)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_period/2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_period/2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_period/2);
    }

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
      uint32_t raw_pa0 = ReadAdcCounts(&hadc1, ADC_CHANNEL_1);
      uint32_t raw_pa6 = ReadAdcCounts(&hadc2, ADC_CHANNEL_3);
      uint32_t raw_pa7 = ReadAdcCounts(&hadc2, ADC_CHANNEL_4);
      uint32_t raw_opamp = ReadAdcCounts(&hadc1, OPAMP_ADC_CHANNEL);

      uint32_t mv_pa0 = AdcCountsToMv(raw_pa0);
      uint32_t mv_pa6 = AdcCountsToMv(raw_pa6);
      uint32_t mv_pa7 = AdcCountsToMv(raw_pa7);
      uint32_t mv_opamp = AdcCountsToMv(raw_opamp);
      uint32_t mv_bus = mv_opamp * 21U;

      UpdateMovingAverage(mv_pa0, mv_pa6, mv_pa7, mv_bus);

      if (avg_count > 0U)
      {
        uint32_t avg_ia_mv = ia_mv_sum / avg_count;
        uint32_t avg_ib_mv = ib_mv_sum / avg_count;
        uint32_t avg_ic_mv = ic_mv_sum / avg_count;
        uint32_t avg_bus_mv = bus_mv_sum / avg_count;

        int32_t ia_ma = ((int32_t)avg_ia_mv - (int32_t)ia_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
        int32_t ib_ma = ((int32_t)avg_ib_mv - (int32_t)ib_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);
        int32_t ic_ma = ((int32_t)avg_ic_mv - (int32_t)ic_zero_mv) * (1000 / CURRENT_SENSE_MV_PER_A);

        uint32_t bus_v = avg_bus_mv / 1000U;
        uint32_t bus_mv_rem = avg_bus_mv % 1000U;

        // Convert floats to integers for printf
        int32_t deg_int = (int32_t)encoder_angle_deg;
        int32_t deg_frac = (int32_t)((encoder_angle_deg - (float)deg_int) * 10.0f);
        int32_t speed_int = (int32_t)encoder_speed;
        int32_t speed_frac = (int32_t)((encoder_speed - (float)speed_int) * 100.0f);
        if (speed_frac < 0) speed_frac = -speed_frac;

        if (open_loop_test)
        {
          printf("OPEN-LOOP Motor:%s Duty:%2lu%% SimAngle:%5u | Iu:%5ldmA Iv:%5ldmA Iw:%5ldmA Bus:%3lu.%03luV\r\n",
                 motor_enabled ? "ON " : "OFF",
                 motor_duty_percent,
                 open_loop_angle,
                 ia_ma, ib_ma, ic_ma, bus_v, bus_mv_rem);
        }
        else
        {
          printf("Motor:%s Dir:%s Duty:%2lu%% | Ang:%5u(%ld.%1lddeg) Spd:%ld.%02ldrad/s | Iu:%5ldmA Iv:%5ldmA Iw:%5ldmA Bus:%3lu.%03luV\r\n",
                 motor_enabled ? "ON " : "OFF", motor_direction > 0 ? "FWD" : "REV",
                 motor_duty_percent,
                 encoder_angle_raw, deg_int, deg_frac, speed_int, speed_frac,
                 ia_ma, ib_ma, ic_ma, bus_v, bus_mv_rem);
        }
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
