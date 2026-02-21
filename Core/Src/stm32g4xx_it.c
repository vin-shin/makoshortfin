#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "config.h"
#include "FOC.h"
#include "encoder.h"
#include "uart_telem.h"
#include <math.h>

/* Shared state (defined in main.c) */
extern volatile uint32_t g_adc_dual_raw[2];
extern volatile uint32_t g_ia_zero_counts;
extern volatile uint32_t g_ib_zero_counts;
extern volatile uint32_t g_ic_zero_counts;
extern volatile uint32_t g_vdda_mv;
extern volatile float g_bus_voltage;
extern volatile uint8_t g_foc_enabled;
extern volatile uint8_t g_fault_code;
extern volatile float g_mech_angle;
extern volatile float g_elec_angle;
extern volatile float g_id_meas;
extern volatile float g_iq_meas;
extern volatile float g_vd;
extern volatile float g_vq;
extern volatile float g_iq_ref;
extern volatile uint32_t g_isr_count;
extern volatile uint32_t g_isr_max_cycles;
extern uint16_t g_encoder_offset_counts;

/* ISR-local filter state */
static float s_ia_f = 0.0f;
static float s_ib_f = 0.0f;
static float s_ic_f = 0.0f;

/* Expose filtered values for diagnostics */
volatile float g_ia_filtered = 0.0f;
volatile float g_ib_filtered = 0.0f;
volatile float g_ic_filtered = 0.0f;
static uint8_t s_spi_fail_count = 0;
static uint16_t s_elec_counts_prev = 0;
static float s_elec_speed_f = 0.0f;   /* rad/s, kept float for angle prediction */
static uint8_t s_elec_speed_valid = 0;

/* Pre-calculated ADC conversion constants (updated when VDDA changes) */
static float s_counts_to_amps_adc1 = 0.0f;
static float s_counts_to_amps_adc2 = 0.0f;
static uint32_t s_last_vdda_mv = 0;

/* Diagnostic counters for ADC conversion tracking */
volatile uint32_t g_adc_ready_count = 0;      /* Times ADC JEOS flags were set */
volatile uint32_t g_adc_not_ready_count = 0;  /* Times ADC JEOS flags were NOT set */
volatile uint32_t g_last_ia_raw = 0;
volatile uint32_t g_last_ib_raw = 0;
volatile uint32_t g_last_ic_raw = 0;

/* Update conversion constants when VDDA changes */
static inline void UpdateADCConversionConstants(void)
{
    if (g_vdda_mv != s_last_vdda_mv) {
        /* Pre-calculate: Amps = (counts - zero_counts) * (VDDA / (4096 * sensitivity_mV_per_A)) */
        s_counts_to_amps_adc1 = (float)g_vdda_mv / ((float)ADC_MAX_COUNTS * CURRENT_SENSE_MV_PER_A);
        s_counts_to_amps_adc2 = (float)g_vdda_mv / ((float)ADC2_MAX_COUNTS * CURRENT_SENSE_MV_PER_A);
        s_last_vdda_mv = g_vdda_mv;
    }
}

/* Expose raw ADC samples for noise testing */
volatile uint16_t s_ia_inj_last = 0U;
volatile uint16_t s_ib_inj_last = 0U;
volatile uint16_t s_ic_inj_last = 0U;


/* ===== Cortex-M4 exception handlers ===== */

void NMI_Handler(void) 
{
    /* Enable GPIOC and blink PC7 for NMI indication */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << (7*2))) | (1UL << (7*2));
    while (1) {
        GPIOC->BSRR = (1UL << 7);
        for (volatile int i = 0; i < 100000; i++);
        GPIOC->BSRR = (1UL << (7 + 16));
        for (volatile int i = 0; i < 100000; i++);
    }
}

void HardFault_Handler(void) 
{
    /* Enable GPIOC and blink PC9 for HardFault indication */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << (9*2))) | (1UL << (9*2));
    while (1) {
        GPIOC->BSRR = (1UL << 9);
        for (volatile int i = 0; i < 100000; i++);
        GPIOC->BSRR = (1UL << (9 + 16));
        for (volatile int i = 0; i < 100000; i++);
    }
}

void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ===== USART1 TX interrupt ===== */
void USART1_IRQHandler(void)
{
    UART_IRQHandler();
}

/* ===== FOC ISR: TIM1 Update @ 20kHz ===== */
/* OPTIMIZED TIMING:
 * 1. TIM1_TRGO (=OC4REF) fires ~2us before center -> triggers ADC injected conversions
 * 2. ADC converts 3 channels (~2us with 12.5 cycle sampling + 16x oversampling)
 * 3. TIM1 Update event fires -> triggers this ISR
 * 4. ADC results are ready immediately (no busy-wait needed)
 * 5. ISR reads results, runs FOC math, updates PWM duties
 * Result: Zero CPU cycles wasted waiting for ADC
 */
void TIM1_UP_TIM16_IRQHandler(void)
{
    /* Clear flag immediately */
    LL_TIM_ClearFlag_UPDATE(TIM1);

    /* Profiling start */
    uint32_t cyc_start = SysTick->VAL;

    /* 1. Read encoder (ALWAYS run, even when FOC disabled, for diagnostics) */
    uint16_t mech_raw;
    Encoder_Status_t enc_st = Encoder_ReadAngle(&mech_raw);

    if (enc_st != ENC_OK) {
        s_spi_fail_count++;
        if (s_spi_fail_count >= SPI_MAX_FAILS) {
            g_foc_enabled = 0;
            g_fault_code = 5; /* encoder fault */
            TIM1->CCR1 = TIM1_HALF_PERIOD;
            TIM1->CCR2 = TIM1_HALF_PERIOD;
            TIM1->CCR3 = TIM1_HALF_PERIOD;
            return;
        }
    } else {
        s_spi_fail_count = 0;
    }

    /* Electrical angle in 15-bit counts.
     * mech_raw * pole_pairs gives elec position; & 0x7FFF wraps to [0, 32767].
     * No float multiply, no fmodf — one integer multiply and one AND. */
    uint16_t elec_counts = (uint16_t)(
        ((uint32_t)mech_raw * MOTOR_POLE_PAIRS - (uint32_t)g_encoder_offset_counts) & 0x7FFFu
    );

    /* Speed estimate (rad/s) from signed 15-bit delta — wraps naturally via int16_t cast */
    if (s_elec_speed_valid) {
        int16_t delta = (int16_t)(elec_counts - s_elec_counts_prev);
        float speed = (float)delta * (TWO_PI_F * (float)FOC_ISR_FREQ_HZ / 32768.0f);
        s_elec_speed_f += FOC_ANGLE_SPEED_ALPHA * (speed - s_elec_speed_f);
    } else {
        s_elec_speed_f = 0.0f;
        s_elec_speed_valid = 1U;
    }
    s_elec_counts_prev = elec_counts;

    /* Telemetry: convert counts to radians only for display */
    g_mech_angle = (float)mech_raw   * (TWO_PI_F / 32768.0f);
    g_elec_angle = (float)elec_counts * (TWO_PI_F / 32768.0f);

    /* 2. Read hardware-triggered ADC results (ALWAYS - needed for calibration) */
    static uint16_t s_ia_inj = 0U;
    static uint16_t s_ib_inj = 0U;
    static uint16_t s_ic_inj = 0U;

    if (LL_ADC_IsActiveFlag_JEOS(ADC1) && LL_ADC_IsActiveFlag_JEOS(ADC2)) {
        g_adc_ready_count++;
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOS(ADC2);
        s_ia_inj = (uint16_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
        s_ib_inj = (uint16_t)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
        s_ic_inj = (uint16_t)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);

        /* Expose for calibration and noise testing */
        s_ia_inj_last = s_ia_inj;
        s_ib_inj_last = s_ib_inj;
        s_ic_inj_last = s_ic_inj;

        g_last_ia_raw = s_ia_inj;
        g_last_ib_raw = s_ib_inj;
        g_last_ic_raw = s_ic_inj;
    } else {
        g_adc_not_ready_count++;
    }

    /* 3. Convert counts to amps and filter (ALWAYS) */
    UpdateADCConversionConstants();

#if INVERT_CURRENT_POLARITY
    float ia_raw = -(((float)s_ia_inj - (float)g_ia_zero_counts) * s_counts_to_amps_adc1);
    float ib_raw = -(((float)s_ib_inj - (float)g_ib_zero_counts) * s_counts_to_amps_adc2);
    float ic_raw = -(((float)s_ic_inj - (float)g_ic_zero_counts) * s_counts_to_amps_adc2);
#else
    float ia_raw = ((float)s_ia_inj - (float)g_ia_zero_counts) * s_counts_to_amps_adc1;
    float ib_raw = ((float)s_ib_inj - (float)g_ib_zero_counts) * s_counts_to_amps_adc2;
    float ic_raw = ((float)s_ic_inj - (float)g_ic_zero_counts) * s_counts_to_amps_adc2;
#endif

    s_ia_f += CURRENT_FILTER_ALPHA * (ia_raw - s_ia_f);
    s_ib_f += CURRENT_FILTER_ALPHA * (ib_raw - s_ib_f);
    s_ic_f += CURRENT_FILTER_ALPHA * (ic_raw - s_ic_f);

    g_ia_filtered = s_ia_f;
    g_ib_filtered = s_ib_f;
    g_ic_filtered = s_ic_f;

    /* If FOC disabled, keep neutral PWM and return (ADC was already sampled above) */
    if (!g_foc_enabled && !PWM_ONLY_MODE) {
        TIM1->CCR1 = TIM1_HALF_PERIOD;
        TIM1->CCR2 = TIM1_HALF_PERIOD;
        TIM1->CCR3 = TIM1_HALF_PERIOD;

        uint32_t cyc_end = SysTick->VAL;
        uint32_t elapsed = (cyc_start >= cyc_end)
            ? (cyc_start - cyc_end)
            : (cyc_start + SysTick->LOAD + 1U - cyc_end);
        g_isr_count++;
        if (elapsed > g_isr_max_cycles) g_isr_max_cycles = elapsed;
        return;
    }

    float ia = s_ia_f;
    float ib = s_ib_f;
    float ic = s_ic_f;

    /* 4. Overcurrent protection */
    if (fabsf(ia) > OVERCURRENT_LIMIT_A || fabsf(ib) > OVERCURRENT_LIMIT_A
        || fabsf(ic) > OVERCURRENT_LIMIT_A) {
        g_foc_enabled = 0;
        g_fault_code = 1;
        TIM1->CCR1 = TIM1_HALF_PERIOD;
        TIM1->CCR2 = TIM1_HALF_PERIOD;
        TIM1->CCR3 = TIM1_HALF_PERIOD;
        return;
    }

    /* Angle prediction for phase lead compensation (in counts) */
    uint16_t elec_counts_used = elec_counts;
#if FOC_ANGLE_PREDICT_ENABLE
    if (fabsf(s_elec_speed_f) >= FOC_ANGLE_PREDICT_MIN_SPEED_RAD_S) {
        /* Convert filtered speed (rad/s) to counts/step lead */
        float lead_f = s_elec_speed_f * (32768.0f / (TWO_PI_F * (float)FOC_ISR_FREQ_HZ));
        /* Clamp to ±30° in counts (30/360 * 32768 = 2730) */
        if (lead_f >  2730.0f) lead_f =  2730.0f;
        if (lead_f < -2730.0f) lead_f = -2730.0f;
        elec_counts_used = (uint16_t)((int32_t)elec_counts + (int32_t)lead_f) & 0x7FFF;
    }
#endif

    /* PWM-only mode: report currents but do not run FOC */
    if (!g_foc_enabled && PWM_ONLY_MODE) {
        float sin_e, cos_e;
        uint16_t angle_inv = elec_counts_used;
#if INVERT_ENCODER_ANGLE
        angle_inv = (uint16_t)(32768U - angle_inv) & 0x7FFF;
#endif
        FOC_SinCos(angle_inv, &sin_e, &cos_e);

        float i_sum = (ia + ib + ic) * (1.0f / 3.0f);
        float ia_z = ia - i_sum;
        float ib_z = ib - i_sum;

        const float clarke_k = 0.81649658f; /* sqrt(2/3) */
        float i_alpha = clarke_k * ia_z;
        float i_beta = clarke_k * (ia_z + 2.0f * ib_z) * INV_SQRT3_F;

        g_id_meas = i_alpha * cos_e + i_beta * sin_e;
        g_iq_meas = -i_alpha * sin_e + i_beta * cos_e;
        g_vd = 0.0f;
        g_vq = 0.0f;

        TIM1->CCR1 = TIM1_HALF_PERIOD;
        TIM1->CCR2 = TIM1_HALF_PERIOD;
        TIM1->CCR3 = TIM1_HALF_PERIOD;

        /* Profiling */
        uint32_t cyc_end = SysTick->VAL;
        uint32_t elapsed = (cyc_start >= cyc_end)
            ? (cyc_start - cyc_end)
            : (cyc_start + SysTick->LOAD + 1U - cyc_end);
        g_isr_count++;
        if (elapsed > g_isr_max_cycles) g_isr_max_cycles = elapsed;
        return;
    }

    /* 4. Run FOC */
    FOC_Sensors_t sensors = {
        .ia = ia,
        .ib = ib,
        .ic = ic,
        .bus_v = g_bus_voltage,
        .omega_e = s_elec_speed_f,
        .elec_counts = elec_counts_used,
    };

    FOC_Output_t foc_out;
    FOC_Run(&sensors, &foc_out);

    /* Export telemetry */
    g_id_meas = foc_out.id_meas;
    g_iq_meas = foc_out.iq_meas;
    g_vd = foc_out.vd;
    g_vq = foc_out.vq;

    /* 5. Apply PWM duties */
    uint32_t ccr_u = (uint32_t)(foc_out.duty_u * (float)TIM1_ARR_VALUE);
    uint32_t ccr_v = (uint32_t)(foc_out.duty_v * (float)TIM1_ARR_VALUE);
    uint32_t ccr_w = (uint32_t)(foc_out.duty_w * (float)TIM1_ARR_VALUE);
    if (ccr_u > TIM1_ARR_VALUE) ccr_u = TIM1_ARR_VALUE;
    if (ccr_v > TIM1_ARR_VALUE) ccr_v = TIM1_ARR_VALUE;
    if (ccr_w > TIM1_ARR_VALUE) ccr_w = TIM1_ARR_VALUE;

    TIM1->CCR1 = ccr_u;
    TIM1->CCR2 = ccr_v;
    TIM1->CCR3 = ccr_w;

    /* 6. Profiling */
    uint32_t cyc_end = SysTick->VAL;
    uint32_t elapsed = (cyc_start >= cyc_end)
        ? (cyc_start - cyc_end)
        : (cyc_start + SysTick->LOAD + 1U - cyc_end);
    g_isr_count++;
    if (elapsed > g_isr_max_cycles) g_isr_max_cycles = elapsed;
}
