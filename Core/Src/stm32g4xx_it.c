#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "config.h"
#include "FOC.h"
#include "encoder.h"
#include "uart_telem.h"
#include <math.h>

/* Shared state (defined in main.c) */
extern volatile uint32_t g_adc_dual_raw;
extern volatile uint32_t g_ia_zero_counts;
extern volatile uint32_t g_ib_zero_counts;
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
extern float g_encoder_offset;

/* ISR-local filter state */
static float s_ia_f1 = 0.0f, s_ia_f2 = 0.0f;
static float s_ib_f1 = 0.0f, s_ib_f2 = 0.0f;
static uint8_t s_spi_fail_count = 0;

/* ===== Cortex-M4 exception handlers ===== */

void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
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
void TIM1_UP_TIM16_IRQHandler(void)
{
    /* Clear flag immediately */
    LL_TIM_ClearFlag_UPDATE(TIM1);

    if (!g_foc_enabled) {
        TIM1->CCR1 = TIM1_HALF_PERIOD;
        TIM1->CCR2 = TIM1_HALF_PERIOD;
        TIM1->CCR3 = TIM1_HALF_PERIOD;
        return;
    }

    /* Profiling start */
    uint32_t cyc_start = SysTick->VAL;

    /* 1. Read ADC dual-mode DMA buffer */
    uint32_t adc_raw = g_adc_dual_raw;
    uint16_t ia_counts = (uint16_t)(adc_raw & 0xFFFF);
    uint16_t ib_counts = (uint16_t)(adc_raw >> 16);

    /* Convert counts -> amps */
    float ia_mv = (float)ia_counts * (float)g_vdda_mv / (float)ADC_MAX_COUNTS;
    float ib_mv = (float)ib_counts * (float)g_vdda_mv / (float)ADC_MAX_COUNTS;
    float ia_zero = (float)g_ia_zero_counts * (float)g_vdda_mv / (float)ADC_MAX_COUNTS;
    float ib_zero = (float)g_ib_zero_counts * (float)g_vdda_mv / (float)ADC_MAX_COUNTS;

#if INVERT_CURRENT_POLARITY
    float ia_raw = -((ia_mv - ia_zero) / CURRENT_SENSE_MV_PER_A);
    float ib_raw = -((ib_mv - ib_zero) / CURRENT_SENSE_MV_PER_A);
#else
    float ia_raw = (ia_mv - ia_zero) / CURRENT_SENSE_MV_PER_A;
    float ib_raw = (ib_mv - ib_zero) / CURRENT_SENSE_MV_PER_A;
#endif

    /* 2nd-order IIR filter (two cascaded 1st-order stages) */
    s_ia_f1 += CURRENT_FILTER_ALPHA * (ia_raw - s_ia_f1);
    s_ia_f2 += CURRENT_FILTER_ALPHA * (s_ia_f1 - s_ia_f2);
    s_ib_f1 += CURRENT_FILTER_ALPHA * (ib_raw - s_ib_f1);
    s_ib_f2 += CURRENT_FILTER_ALPHA * (s_ib_f1 - s_ib_f2);

    float ia = s_ia_f2;
    float ib = s_ib_f2;

    /* 2. Overcurrent protection */
    if (fabsf(ia) > OVERCURRENT_LIMIT_A || fabsf(ib) > OVERCURRENT_LIMIT_A) {
        g_foc_enabled = 0;
        g_fault_code = 1;
        TIM1->CCR1 = TIM1_HALF_PERIOD;
        TIM1->CCR2 = TIM1_HALF_PERIOD;
        TIM1->CCR3 = TIM1_HALF_PERIOD;
        return;
    }

    /* 3. Read encoder */
    float mech_angle;
    Encoder_Status_t enc_st = Encoder_ReadAngle(&mech_angle);

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

    /* Compute electrical angle */
    float elec_angle = fmodf(mech_angle * (float)MOTOR_POLE_PAIRS, TWO_PI_F) - g_encoder_offset;
    if (elec_angle < 0.0f) elec_angle += TWO_PI_F;

    g_mech_angle = mech_angle;
    g_elec_angle = elec_angle;

    /* 4. Run FOC */
    FOC_Sensors_t sensors = {
        .ia = ia,
        .ib = ib,
        .bus_v = g_bus_voltage,
        .elec_angle = elec_angle,
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
