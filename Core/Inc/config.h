#ifndef CONFIG_H
#define CONFIG_H

/* ===== Debug Options ===== */
#define ENABLE_WATCHDOG         0       /* Set to 0 to disable watchdog for PCB testing */
#define PWM_ONLY_MODE           0       /* 1 = keep PWM running at 50% with FOC disabled; 0 = enable FOC */
#define FOC_ENABLE_DIAGNOSTICS  0       /* 1 = capture diagnostic variables (adds ISR overhead); 0 = disable for max performance */

/* ===== Motor Parameters ===== */
#define MOTOR_POLE_PAIRS        20U
#define MOTOR_MAX_CURRENT_A     100.0f
#define MOTOR_RATED_CURRENT_A   100.0f

/* ===== FOC Timing ===== */
#define FOC_ISR_FREQ_HZ         20000U
#define FOC_ISR_DT_S            (1.0f / (float)FOC_ISR_FREQ_HZ)
#define TIM1_ARR_VALUE          2125U
#define TIM1_HALF_PERIOD        1062U
#define TIM1_DEADTIME           25U     /* 1 tick = ~5.88 ns. 25 ticks = 147ns */
#define TIM1_RCR_VALUE          3U      /* Update every 4th event = 20kHz */

/* ===== Angle Prediction ===== */
#define FOC_ANGLE_PREDICT_ENABLE        1
#define FOC_ANGLE_PREDICT_DELAY_S       (1.43 * FOC_ISR_DT_S)  /* Full PWM period prediction */
#define FOC_ANGLE_PREDICT_MAX_RAD       0.52359878f  /* 30 deg */
#define FOC_ANGLE_PREDICT_MIN_SPEED_RAD_S 1.0f  /* Activate at low speeds */
#define FOC_ANGLE_SPEED_ALPHA           0.2f

/* ===== Current Loop PI Gains ===== */
#define FOC_KP_D                0.0307f   /* Proportional only - conservative for d-axis stability */
#define FOC_KI_D                35.40f    /* No d-axis integration */
#define FOC_KP_Q                0.0307f   /* Increased 2.5x from 0.1 - faster response, still stable */
#define FOC_KI_Q                35.40f   /* Tiny integrator (0.02) - improves steady-state without windup */


/* ===== Current Sensing ===== */
/* Hardware: ACS72981 5V variant hall effect sensors (no voltage divider)
 * - Supply: 5V, Output: 2.5V @ 0A (VCC/2)
 * - Sensitivity: 20 mV/A (verify from your specific part number)
 * - Output impedance: <1Ω, Bandwidth: 250 kHz
 * - Connected directly to 3.3V ADC inputs
 * WARNING: ADC clipping limits with 2.5V zero-point:
 *   - Positive: (3300mV - 2500mV) / 20mV/A = +40A max before clipping
 *   - Negative: (2500mV - 0mV) / 20mV/A = -125A max before clipping
 *   - Current limits (10A overcurrent) are well within safe range
 * ADC Configuration:
 * - Hardware-triggered injected conversions from TIM1_TRGO (eliminating ISR jitter)
 * - Sampling time: 12.5 cycles @ 42.5MHz = 294ns (optimal for hall sensors)
 * - 16x oversampling with right-shift 4 (effective 12-bit with reduced noise)
 * - Total conversion time: ~600ns per channel
 */
#define ADC_VREF_NOM_MV         3300U
#define ADC_MAX_COUNTS          4095U
#define ADC2_MAX_COUNTS         4095U  /* Observed dual DMA samples are 12-bit for ADC2 */
#define CURRENT_SENSE_MV_PER_A  20.0f  /* ACS72981 sensitivity - verify your part number! */
#define CURRENT_ZERO_NOM_MV     2500U  /* 5V variant: VCC/2 = 2.5V at zero current */
#define CURRENT_FILTER_ALPHA    0.7f   /* IIR filter: fc~3800Hz, phase lag ~1.5° @ 100Hz, ~3° @ 200Hz */
#define INVERT_CURRENT_POLARITY 1       /* 0 = don't negate measured current */
#define INVERT_ENCODER_ANGLE    0       /* 1 = negate encoder angle to correct rotation direction */
#define ZERO_CURRENT_THRESHOLD  0.1f    /* A - drift correction active below this current */
#define ZERO_DRIFT_ALPHA        0.00001f /* Drift integrator alpha, ~5s per 0.1A error */
#define VDDA_RECALIB_MS         10000U  /* Re-calibrate VDDA every 10 seconds */

/* ===== Bus Voltage ===== */
#define BUS_V_DIVIDER_RATIO     21.0f   /* Hardware: 200k top + 10k bottom = 21x divider */
#define BUS_V_MIN               20.0f   /* Minimum bus voltage (24V nominal - 4V margin) */
#define BUS_V_MAX               28.0f   /* Maximum bus voltage (24V nominal + 4V margin) */
#define BUS_V_UPDATE_MS         1000U
#define BUS_V_FILTER_ALPHA      0.2f
#define BUS_V_OUTLIER_THRESH    2.0f    /* Reject readings >2V from filtered */

/* ===== Encoder (A1333) ===== */
#define ENCODER_BITS            15U
#define ENCODER_COUNTS          32768U
#define ENCODER_TWO_PI          6.28318530718f
#define A1333_ANG15_CMD         0x3200U
#define A1333_NOP_CMD           0x0000U
#define SPI_TIMEOUT_LOOPS       5100U   /* ~30us @ 170MHz */
#define SPI_MAX_FAILS           5U

/* ===== Rotor Alignment ===== */
#define ALIGN_VOLTAGE_V         1.0f
#define ALIGN_TIME_MS           800U    /* Reduced from 1000ms for faster startup */
#define ALIGN_RAMPDOWN_MS       200U

/* ===== Safety ===== */
#define OVERCURRENT_LIMIT_A     100.0f  /* DISABLED for tuning - set high */

/* ===== UART Telemetry ===== */
#define UART_TX_BUF_SIZE        256U
#define UART_BAUD               115200U
#define TELEMETRY_PERIOD_MS     20U

/* ===== Pin Assignments ===== */
/* TIM1 PWM (AF6) */
#define PWM_UH_PORT             GPIOA
#define PWM_UH_PIN              LL_GPIO_PIN_8
#define PWM_VH_PORT             GPIOA
#define PWM_VH_PIN              LL_GPIO_PIN_9
#define PWM_WH_PORT             GPIOA
#define PWM_WH_PIN              LL_GPIO_PIN_10
#define PWM_UL_PORT             GPIOA
#define PWM_UL_PIN              LL_GPIO_PIN_11
#define PWM_VL_PORT             GPIOA
#define PWM_VL_PIN              LL_GPIO_PIN_12
#define PWM_WL_PORT             GPIOB
#define PWM_WL_PIN              LL_GPIO_PIN_15

/* ADC current sense (analog) */
#define ISENSE_A_PORT           GPIOA
#define ISENSE_A_PIN            LL_GPIO_PIN_0   /* ADC1_IN1 */
#define ISENSE_B_PORT           GPIOA
#define ISENSE_B_PIN            LL_GPIO_PIN_6   /* ADC2_IN3 */
#define ISENSE_C_PORT           GPIOA
#define ISENSE_C_PIN            LL_GPIO_PIN_7   /* ADC2_IN4 */

/* OPAMP3 bus voltage */
#define VBUS_INP_PORT           GPIOB
#define VBUS_INP_PIN            LL_GPIO_PIN_0   /* OPAMP3_VINP */
#define VBUS_OUT_PORT           GPIOB
#define VBUS_OUT_PIN            LL_GPIO_PIN_1   /* OPAMP3_VOUT */

/* SPI3 encoder (AF6) */
#define ENC_SCK_PORT            GPIOC
#define ENC_SCK_PIN             LL_GPIO_PIN_10
#define ENC_MISO_PORT           GPIOC
#define ENC_MISO_PIN            LL_GPIO_PIN_11
#define ENC_MOSI_PORT           GPIOC
#define ENC_MOSI_PIN            LL_GPIO_PIN_12
#define ENC_CS_PORT             GPIOA
#define ENC_CS_PIN              LL_GPIO_PIN_15  /* manual GPIO */

/* USART1 TX (AF7) */
#define UART_TX_PORT            GPIOB
#define UART_TX_PIN             LL_GPIO_PIN_6

/* Gate driver enables */
#define GATE_EN1_PORT           GPIOC
#define GATE_EN1_PIN            LL_GPIO_PIN_7
#define GATE_EN2_PORT           GPIOC
#define GATE_EN2_PIN            LL_GPIO_PIN_8
#define GATE_EN3_PORT           GPIOC
#define GATE_EN3_PIN            LL_GPIO_PIN_9

/* FDCAN1 (AF9, placeholder) */
#define FDCAN_RX_PORT           GPIOB
#define FDCAN_RX_PIN            LL_GPIO_PIN_8
#define FDCAN_TX_PORT           GPIOB
#define FDCAN_TX_PIN            LL_GPIO_PIN_9

/* ===== Math Constants ===== */
#define PI_F                    3.14159265358979f
#define TWO_PI_F                6.28318530718f
#define INV_SQRT3_F             0.57735026919f
#define SQRT3_F                 1.73205080757f
#define SVPWM_V_LIMIT           0.57735026919f  /* 1/sqrt(3) */
#define RAD_TO_DEG_F            57.2957795131f
#define DEG_TO_RAD_F            0.01745329252f

#endif /* CONFIG_H */
