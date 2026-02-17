#include "main.h"
#include "config.h"
#include "periph_ll.h"
#include "FOC.h"
#include "encoder.h"
#include "control.h"
#include "uart_telem.h"
#include <math.h>
#include <string.h>

/* ===== Global shared state ===== */

/* ADC DMA destination (written by DMA, read by ISR) */
volatile uint32_t g_adc_dual_raw = 0;

/* Current zero calibration */
volatile uint32_t g_ia_zero_counts = 2048;
volatile uint32_t g_ib_zero_counts = 2048;

/* Calibrated VDDA in millivolts */
volatile uint32_t g_vdda_mv = ADC_VREF_NOM_MV;

/* Bus voltage (updated in main loop, read by ISR) */
volatile float g_bus_voltage = 0.0f;

/* FOC enable / fault */
volatile uint8_t g_foc_enabled = 0;
volatile uint8_t g_fault_code = 0;

/* ISR -> main telemetry */
volatile float g_mech_angle = 0.0f;
volatile float g_elec_angle = 0.0f;
volatile float g_id_meas = 0.0f;
volatile float g_iq_meas = 0.0f;
volatile float g_vd = 0.0f;
volatile float g_vq = 0.0f;
volatile float g_iq_ref = 0.0f;

/* ISR profiling */
volatile uint32_t g_isr_count = 0;
volatile uint32_t g_isr_max_cycles = 0;

/* Encoder offset (set during calibration, read by ISR) */
float g_encoder_offset = 0.0f;

/* Position target (read by uart_telem for telemetry) */
float g_pos_target = 0.0f;

/* ===== VDDA calibration via internal VREFINT ===== */
static void CalibrateVDDA(void)
{
    /* VREFINT factory cal was measured at 3.0V, stored at 0x1FFF75AA */
    uint16_t vrefint_cal = *((uint16_t *)0x1FFF75AA);

    /* Configure ADC1 injected to read VREFINT (CH18) */
    /* Save current injected channel config */
    uint32_t saved_jsqr = ADC1->JSQR;

    /* Set injected channel to VREFINT, software trigger */
    ADC1->JSQR = (LL_ADC_CHANNEL_VREFINT << ADC_JSQR_JSQ1_Pos);

    /* Set long sampling time for VREFINT (247.5 cycles) */
    /* VREFINT is on CH18 -> SMPR2 bits [26:24] */
    uint32_t saved_smpr2 = ADC1->SMPR2;
    ADC1->SMPR2 = (ADC1->SMPR2 & ~(0x7UL << 24)) | (0x7UL << 24); /* 247.5 cyc */

    /* Enable VREFINT */
    ADC12_COMMON->CCR |= ADC_CCR_VREFEN;

    /* Average 16 readings */
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        LL_ADC_INJ_StartConversion(ADC1);
        while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) {}
        LL_ADC_ClearFlag_JEOS(ADC1);
        sum += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    }
    uint32_t vrefint_raw = sum / 16;

    /* Restore */
    ADC12_COMMON->CCR &= ~ADC_CCR_VREFEN;
    ADC1->SMPR2 = saved_smpr2;
    ADC1->JSQR = saved_jsqr;

    /* Calculate true VDDA: VDDA = 3000mV * CAL / RAW */
    if (vrefint_raw > 0) {
        g_vdda_mv = (3000UL * vrefint_cal) / vrefint_raw;
    }
}

/* ===== Bus voltage reading (blocking, for init only) ===== */
static float ReadBusVoltageBlocking(void)
{
    /* Injected ADC1 CH12 (OPAMP3 output) */
    LL_ADC_INJ_StartConversion(ADC1);
    while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) {}
    LL_ADC_ClearFlag_JEOS(ADC1);

    uint32_t raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    float voltage = (float)raw * (float)g_vdda_mv / ((float)ADC_MAX_COUNTS * 1000.0f);
    return voltage * BUS_V_DIVIDER_RATIO;
}

/* ===== Open-loop SVPWM for rotor alignment ===== */
static void AlignSVPWM(float vd, float vq, float elec_angle, float bus_v)
{
    float sin_e, cos_e;
    FOC_SinCos(elec_angle, &sin_e, &cos_e);

    float v_alpha = vd * cos_e - vq * sin_e;
    float v_beta = vd * sin_e + vq * cos_e;

    float vu = v_alpha;
    float vv = -0.5f * v_alpha + 0.5f * SQRT3_F * v_beta;
    float vw = -0.5f * v_alpha - 0.5f * SQRT3_F * v_beta;

    float v_max = fmaxf(vu, fmaxf(vv, vw));
    float v_min = fminf(vu, fminf(vv, vw));
    float v_off = 0.5f * (v_max + v_min);
    vu -= v_off;
    vv -= v_off;
    vw -= v_off;

    float inv_bus = 1.0f / bus_v;
    float du = 0.5f + vu * inv_bus;
    float dv = 0.5f + vv * inv_bus;
    float dw = 0.5f + vw * inv_bus;

    if (du < 0.0f) du = 0.0f; else if (du > 1.0f) du = 1.0f;
    if (dv < 0.0f) dv = 0.0f; else if (dv > 1.0f) dv = 1.0f;
    if (dw < 0.0f) dw = 0.0f; else if (dw > 1.0f) dw = 1.0f;

    TIM1->CCR1 = (uint32_t)(du * (float)TIM1_ARR_VALUE);
    TIM1->CCR2 = (uint32_t)(dv * (float)TIM1_ARR_VALUE);
    TIM1->CCR3 = (uint32_t)(dw * (float)TIM1_ARR_VALUE);
}

/* ===== Encoder offset calibration ===== */
static float CalibrateEncoderOffset(float bus_v)
{
    /* Apply d-axis voltage at electrical angle 0 to lock rotor */
    AlignSVPWM(ALIGN_VOLTAGE_V, 0.0f, 0.0f, bus_v);
    HAL_Delay(ALIGN_TIME_MS);

    /* Average encoder readings at locked position */
    float sum = 0.0f;
    for (int i = 0; i < 16; i++) {
        float angle;
        Encoder_ReadAngle(&angle);
        sum += angle;
        HAL_Delay(5);
    }
    float mech_at_zero = sum / 16.0f;

    /* Electrical offset = mechanical * pole_pairs, wrapped to [0, 2pi) */
    float offset = fmodf(mech_at_zero * (float)MOTOR_POLE_PAIRS, TWO_PI_F);
    if (offset < 0.0f) offset += TWO_PI_F;

    UART_Printf("Encoder offset: mech=%.1fdeg elec=%.1fdeg\r\n",
                 mech_at_zero * RAD_TO_DEG_F, offset * RAD_TO_DEG_F);

    /* Ramp down alignment voltage */
    for (int i = 0; i < 22; i++) {
        float v = ALIGN_VOLTAGE_V * (1.0f - (float)i / 22.0f);
        AlignSVPWM(v, 0.0f, 0.0f, bus_v);
        HAL_Delay(10);
    }

    /* Zero voltage */
    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;

    return offset;
}

/* ===== Current zero calibration ===== */
static void CalibrateCurrentZeros(void)
{
    uint32_t sum_ia = 0, sum_ib = 0;
    for (int i = 0; i < 500; i++) {
        uint32_t raw = g_adc_dual_raw;
        sum_ia += (raw & 0xFFFF);
        sum_ib += (raw >> 16);
        HAL_Delay(2);
    }
    g_ia_zero_counts = sum_ia / 500;
    g_ib_zero_counts = sum_ib / 500;

    UART_Printf("Current zeros: ia=%lu ib=%lu counts\r\n",
                 (unsigned long)g_ia_zero_counts,
                 (unsigned long)g_ib_zero_counts);
}

/* ===== Bus voltage non-blocking state machine ===== */
typedef enum { BV_IDLE, BV_WAIT } BusV_State_t;
static BusV_State_t s_bv_state = BV_IDLE;
static float s_bv_filtered = 0.0f;

static void BusVoltage_Update(void)
{
    switch (s_bv_state) {
    case BV_IDLE:
        /* Start injected conversion (avoid collision with TIM1 trigger) */
        if (TIM1->CNT > 200 && TIM1->CNT < 1800) {
            LL_ADC_INJ_StartConversion(ADC1);
            s_bv_state = BV_WAIT;
        }
        break;

    case BV_WAIT:
        if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
            LL_ADC_ClearFlag_JEOS(ADC1);
            uint32_t raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
            float v = (float)raw * (float)g_vdda_mv
                    / ((float)ADC_MAX_COUNTS * 1000.0f) * BUS_V_DIVIDER_RATIO;

            /* Outlier rejection + low-pass filter */
            if (s_bv_filtered == 0.0f) {
                s_bv_filtered = v; /* First reading */
            } else if (fabsf(v - s_bv_filtered) < BUS_V_OUTLIER_THRESH) {
                s_bv_filtered += BUS_V_FILTER_ALPHA * (v - s_bv_filtered);
            }
            /* else: outlier, skip */

            g_bus_voltage = s_bv_filtered;

            /* Safety check */
            if (s_bv_filtered > BUS_V_MAX) {
                g_foc_enabled = 0;
                g_fault_code = 2; /* overvoltage */
            } else if (s_bv_filtered < BUS_V_MIN && s_bv_filtered > 1.0f) {
                g_foc_enabled = 0;
                g_fault_code = 3; /* undervoltage */
            }

            s_bv_state = BV_IDLE;
        }
        break;
    }
}

/* ===== MAIN ===== */
int main(void)
{
    /* 1. HAL init (SysTick only) */
    HAL_Init();

    /* 2. System clock 170MHz */
    LL_SystemClock_Init();

    /* Update HAL's SystemCoreClock for HAL_Delay */
    SystemCoreClock = 170000000UL;

    /* 3. All GPIO */
    LL_GPIO_Init_All();

    /* 4. OPAMP3 (bus voltage buffer) */
    LL_OPAMP3_Init();

    /* 5. DMA for ADC */
    LL_DMA_Init_ADC(&g_adc_dual_raw);

    /* 6. ADC1+ADC2 dual mode */
    LL_ADC_Init_All();
    LL_ADC_Calibrate_All();

    /* 7. TIM1 PWM (counter starts, PWM channels enabled) */
    LL_TIM1_Init();

    /* 8. SPI3 for encoder */
    LL_SPI3_Init();

    /* 9. USART1 for telemetry */
    LL_USART1_Init();

    /* 10. FDCAN placeholder */
    LL_FDCAN1_Init();

    /* 11. Enable gate drivers */
    LL_GPIO_SetOutputPin(GATE_EN1_PORT, GATE_EN1_PIN);
    LL_GPIO_SetOutputPin(GATE_EN2_PORT, GATE_EN2_PIN);
    LL_GPIO_SetOutputPin(GATE_EN3_PORT, GATE_EN3_PIN);

    /* 12. Software init */
    FOC_Init();
    FOC_SetGains(FOC_KP_D, FOC_KI_D, FOC_KP_Q, FOC_KI_Q);
    FOC_SetCurrentRefs(0.0f, 0.0f);

    Encoder_Init();
    Control_Init();
    Control_SetGains(POS_KP, POS_KI, POS_KD);
    UART_Init();

    UART_Printf("\r\nMako Shortfin FOC v2.0\r\n");
    UART_Printf("Clock: %lu MHz\r\n", (unsigned long)(SystemCoreClock / 1000000UL));

    /* 13. Calibrate VDDA */
    LL_ADC_Start_All();
    HAL_Delay(10);
    CalibrateVDDA();
    UART_Printf("VDDA: %lu mV\r\n", (unsigned long)g_vdda_mv);

    /* 14. Enable PWM outputs at 50% duty (zero voltage) */
    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;
    LL_TIM_EnableAllOutputs(TIM1);

    /* 15. Read initial bus voltage */
    float bus_v = ReadBusVoltageBlocking();
    g_bus_voltage = bus_v;
    s_bv_filtered = bus_v;
    UART_Printf("Bus: %ld mV\r\n", (long)(bus_v * 1000.0f));

    /* 16. Encoder offset calibration */
    g_encoder_offset = CalibrateEncoderOffset(bus_v);

    /* 17. Current zero calibration (motor de-energized) */
    HAL_Delay(500);
    CalibrateCurrentZeros();

    /* 18. Prepare FOC */
    FOC_Reset();
    FOC_SetCurrentRefs(0.0f, 0.0f);

    /* 19. Start watchdog */
    LL_IWDG_Init();

    /* 20. Enable FOC ISR */
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    LL_TIM_EnableIT_UPDATE(TIM1);
    g_foc_enabled = 1;

    UART_Printf("FOC running @ %lu Hz\r\n", (unsigned long)FOC_ISR_FREQ_HZ);

    /* ===== Main loop ===== */
    uint32_t last_pos_ms = HAL_GetTick();
    uint32_t last_telem_ms = HAL_GetTick();
    uint32_t last_busv_ms = HAL_GetTick();
    uint32_t last_switch_ms = HAL_GetTick();
    uint8_t target_select = 0;

    g_pos_target = POS_TARGET_A_DEG * DEG_TO_RAD_F;
    Control_SetTarget(g_pos_target);

    while (1) {
        uint32_t now = HAL_GetTick();

        /* Position control @ 1kHz */
        if (now - last_pos_ms >= 1) {
            float dt = (float)(now - last_pos_ms) * 0.001f;
            last_pos_ms = now;

            /* Toggle target every POS_SWITCH_MS */
            if (now - last_switch_ms >= POS_SWITCH_MS) {
                last_switch_ms = now;
                target_select ^= 1;
                float tgt_deg = target_select ? POS_TARGET_B_DEG : POS_TARGET_A_DEG;
                g_pos_target = tgt_deg * DEG_TO_RAD_F;
                Control_SetTarget(g_pos_target);
                Control_Reset();
            }

            float iq_cmd = Control_Update(g_mech_angle, dt);
            g_iq_ref = iq_cmd;
            FOC_SetCurrentRefs(0.0f, iq_cmd);
        }

        /* Bus voltage @ periodic */
        if (now - last_busv_ms >= BUS_V_UPDATE_MS) {
            last_busv_ms = now;
            BusVoltage_Update();
        }
        /* Also check if conversion is pending */
        if (s_bv_state == BV_WAIT) {
            BusVoltage_Update();
        }

        /* Telemetry @ 50Hz */
        if (now - last_telem_ms >= TELEMETRY_PERIOD_MS) {
            last_telem_ms = now;
            UART_SendTelemetry();
        }

        /* Watchdog refresh */
        IWDG->KR = 0xAAAA;
    }
}

/* ===== System Clock Config (HAL compatibility - called by HAL_Init) ===== */
void SystemClock_Config(void)
{
    /* Clock is configured by LL_SystemClock_Init() instead */
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
