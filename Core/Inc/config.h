#ifndef CONFIG_H
#define CONFIG_H

/* ===== Motor Parameters ===== */
#define MOTOR_POLE_PAIRS        20U
#define MOTOR_MAX_CURRENT_A     8.0f
#define MOTOR_RATED_CURRENT_A   3.0f

/* ===== FOC Timing ===== */
#define FOC_ISR_FREQ_HZ         20000U
#define FOC_ISR_DT_S            (1.0f / (float)FOC_ISR_FREQ_HZ)
#define TIM1_ARR_VALUE          2125U
#define TIM1_HALF_PERIOD        1062U
#define TIM1_DEADTIME           38U     /* ~223ns @ 170MHz */
#define TIM1_RCR_VALUE          3U      /* Update every 4th event = 20kHz */

/* ===== Current Loop PI Gains ===== */
#define FOC_KP_D                0.25f
#define FOC_KI_D                2.0f
#define FOC_KP_Q                0.25f
#define FOC_KI_Q                2.0f

/* ===== Position Loop PID Gains ===== */
#define POS_KP                  0.5f
#define POS_KI                  1.2f
#define POS_KD                  0.02f
#define POS_IQ_LIMIT_A          2.5f
#define POS_KI_LIMIT_A          2.5f
#define POS_LOOP_HZ             1000U
#define POS_DERIV_ALPHA         0.06f   /* LP filter for derivative, fc~10Hz @ 1kHz */

/* ===== Current Sensing ===== */
#define ADC_VREF_NOM_MV         3300U
#define ADC_MAX_COUNTS          4095U
#define CURRENT_SENSE_MV_PER_A  20.0f
#define CURRENT_ZERO_NOM_MV     2500U
#define CURRENT_FILTER_ALPHA    0.15f   /* IIR alpha, fc~500Hz @ 20kHz */
#define INVERT_CURRENT_POLARITY 1       /* 1 = negate measured current */

/* ===== Bus Voltage ===== */
#define BUS_V_DIVIDER_RATIO     17.0f
#define BUS_V_MIN               8.0f
#define BUS_V_MAX               50.0f
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
#define ALIGN_VOLTAGE_V         6.0f
#define ALIGN_TIME_MS           1000U
#define ALIGN_RAMPDOWN_MS       220U

/* ===== Safety ===== */
#define OVERCURRENT_LIMIT_A     8.0f

/* ===== UART Telemetry ===== */
#define UART_TX_BUF_SIZE        256U
#define UART_BAUD               115200U
#define TELEMETRY_PERIOD_MS     20U

/* ===== Position Control Startup ===== */
#define POS_TARGET_A_DEG        0.0f
#define POS_TARGET_B_DEG        180.0f
#define POS_SWITCH_MS           2000U

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
