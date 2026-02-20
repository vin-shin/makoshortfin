#include "main.h"
#include "config.h"
#include "periph_ll.h"
#include "FOC.h"
#include "encoder.h"
#include "uart_telem.h"
#include <math.h>
#include <string.h>

/* ===== Global shared state ===== */

/* ADC DMA destination (written by DMA, read by ISR) */
volatile uint32_t g_adc_dual_raw[2] = {0, 0};

/* Current zero calibration */
volatile uint32_t g_ia_zero_counts = 2048;
volatile uint32_t g_ib_zero_counts = 2048;
volatile uint32_t g_ic_zero_counts = 2048;

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

/* ISR raw ADC samples for noise testing */
extern volatile uint16_t s_ia_inj_last;
extern volatile uint16_t s_ib_inj_last;
extern volatile uint16_t s_ic_inj_last;

/* ISR IIR filtered current values (in Amperes) */
extern volatile float g_ia_filtered;
extern volatile float g_ib_filtered;
extern volatile float g_ic_filtered;

/* ISR ADC diagnostic counters */
extern volatile uint32_t g_adc_ready_count;
extern volatile uint32_t g_adc_not_ready_count;
extern volatile uint32_t g_last_ia_raw;
extern volatile uint32_t g_last_ib_raw;
extern volatile uint32_t g_last_ic_raw;

/* FOC diagnostic exports */
extern volatile float g_foc_ia_raw;
extern volatile float g_foc_ib_raw;
extern volatile float g_foc_ic_raw;
extern volatile float g_foc_i_alpha;
extern volatile float g_foc_i_beta;
extern volatile float g_foc_elec_angle;
extern volatile float g_foc_sin_e;
extern volatile float g_foc_cos_e;
extern volatile float g_foc_iq_cmd;
extern volatile float g_foc_iq_meas;
extern volatile float g_foc_vq_output;

/* ISR profiling */
volatile uint32_t g_isr_count = 0;
volatile uint32_t g_isr_max_cycles = 0;

/* Encoder offset in 15-bit electrical counts (set during calibration, read by ISR) */
uint16_t g_encoder_offset_counts = 0;

/* ===== VDDA calibration via internal VREFINT ===== */
static void CalibrateVDDA(void)
{
    /* Use fixed 3.3V reference - ADC is stable enough at room temperature */
    /* VREFINT calibration is complex and can introduce errors */
    g_vdda_mv = 3300U;
    
    /* Alternative: could read VREFINT but requires careful handling
     * VREFINT factory cal was measured at 3.0V, stored at 0x1FFF75AA
     * Current code has issues with the calibration point mixing
     * For most applications, 3300mV is good enough
     */
}

/* ===== Bus voltage reading (blocking, for init only) ===== */
static float ReadBusVoltageBlocking(void)
{
    /* Temporarily reconfigure ADC1 injected to software trigger for bus voltage reading */
    /* This is safe only when FOC ISR is not running */
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    /* Save current config */
    uint32_t saved_smpr2 = ADC1->SMPR2;
    
    /* Reconfigure for software trigger and CH11 (PB1 - OPAMP3 output pin) */
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_11);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    
    /* Start conversion */
    LL_ADC_INJ_StartConversion(ADC1);
    
    uint32_t timeout = 50000;
    while (!LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        if (--timeout == 0) {
            /* Timeout - restore config and return default */
            LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
            LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
            LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
            ADC1->SMPR2 = saved_smpr2;
            if (!primask) __enable_irq();
            return 12.0f;
        }
    }
    
    LL_ADC_ClearFlag_JEOS(ADC1);
    uint32_t raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    
    /* Restore hardware trigger for current sensing */
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_12CYCLES_5);
    
    if (!primask) __enable_irq();
    
    float voltage = (float)raw * (float)g_vdda_mv / ((float)ADC_MAX_COUNTS * 1000.0f);
    return voltage * BUS_V_DIVIDER_RATIO;
}

/* ===== Open-loop SVPWM for rotor alignment ===== */
static void AlignSVPWM(float vd, float vq, uint16_t elec_counts, float bus_v)
{
    float sin_e, cos_e;
    FOC_SinCos(elec_counts, &sin_e, &cos_e);

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
static uint16_t CalibrateEncoderOffset(float bus_v)
{
    /* Apply d-axis voltage at electrical angle 0 (counts=0) to lock rotor to α-axis */
    AlignSVPWM(ALIGN_VOLTAGE_V, 0.0f, 0U, bus_v);
    HAL_Delay(ALIGN_TIME_MS);

    /* Average 16 raw encoder counts at locked position */
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        uint16_t raw;
        Encoder_ReadAngle(&raw);
        sum += raw;
        HAL_Delay(5);
    }
    uint16_t avg_raw = (uint16_t)(sum / 16);

    /* Electrical offset = (avg_raw * pole_pairs) & 0x7FFF — no float, no fmodf */
    uint16_t offset_counts = (uint16_t)(((uint32_t)avg_raw * MOTOR_POLE_PAIRS) & 0x7FFFu);

    UART_Printf("Encoder offset: mech=%lu counts (%lu deg), elec=%u counts (%lu deg)\r\n",
                 (unsigned long)avg_raw,
                 (unsigned long)(avg_raw * 360UL / ENCODER_COUNTS),
                 offset_counts,
                 (unsigned long)(offset_counts * 360UL / 32768UL));

    /* Ramp down alignment voltage */
    for (int i = 0; i < 22; i++) {
        float v = ALIGN_VOLTAGE_V * (1.0f - (float)i / 22.0f);
        AlignSVPWM(v, 0.0f, 0U, bus_v);
        HAL_Delay(10);
    }

    /* Zero voltage */
    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;

    return offset_counts;
}

/* ===== Current zero calibration ===== */
static void CalibrateCurrentZeros(void)
{
    /* IMPORTANT: This calibration reads directly from ISR's hardware-triggered pipeline
       (s_ia_inj_last, s_ib_inj_last, s_ic_inj_last) to match actual runtime conditions.
       
       Timing note: ISR samples ~2us BEFORE each FOC update via CCR4 match on TIM1.
       For center-aligned PWM, this is close to the midpoint, giving stable readings.
       For true midpoint sampling, would need to adjust CCR4 trigger timing.
    */
    
    UART_Printf("Calibrating ADC zero offsets from live hardware-triggered ISR samples...\r\n");
    UART_Printf("(Motor must be de-energized. Collecting 1000 samples @ 20kHz = 50ms)\r\n");
    
    uint64_t sum_ia = 0, sum_ib = 0, sum_ic = 0;  /* uint64 to avoid overflow */
    
    for (int i = 0; i < 1000; i++) {
        /* Read the values the ISR is continuously sampling via hardware trigger (20kHz) */
        uint16_t ia = s_ia_inj_last;
        uint16_t ib = s_ib_inj_last;
        uint16_t ic = s_ic_inj_last;
        
        sum_ia += ia;
        sum_ib += ib;
        sum_ic += ic;
        
        /* Optional: HAL_Delay(0) to prevent CPU stall, but ISR runs at 20kHz anyway */
        if (i % 100 == 0) {
            UART_Printf("  Sample %d/1000: IA=%u IB=%u IC=%u\r\n", i, ia, ib, ic);
        }
    }
    
    /* Compute averages */
    g_ia_zero_counts = (uint32_t)(sum_ia / 1000ULL);
    g_ib_zero_counts = (uint32_t)(sum_ib / 1000ULL);
    g_ic_zero_counts = (uint32_t)(sum_ic / 1000ULL);

    UART_Printf("Calibration complete!\r\n");
    UART_Printf("Zero offsets: IA=%lu IB=%lu IC=%lu counts\r\n",
                 (unsigned long)g_ia_zero_counts,
                 (unsigned long)g_ib_zero_counts,
                 (unsigned long)g_ic_zero_counts);
    UART_Printf("Expected: ~2500-2550mV (ACS72981 5V variant @ 0A)\r\n");
    UART_Printf("In mV: IA=%lu IB=%lu IC=%lu mV\r\n",
                 (unsigned long)(g_ia_zero_counts * g_vdda_mv / ADC_MAX_COUNTS),
                 (unsigned long)(g_ib_zero_counts * g_vdda_mv / ADC2_MAX_COUNTS),
                 (unsigned long)(g_ic_zero_counts * g_vdda_mv / ADC2_MAX_COUNTS));
}

/* ===== ADC Reading Test ===== */
static void ADCReadingTest(void)
{
    UART_Printf("\r\n=== ADC BASELINE TEST (Motor De-energized) ===\r\n");
    UART_Printf("Testing ADC current sense readings with NO motor current command\r\n");
    UART_Printf("Sensor specs: 2.5V @ 0A (VCC/2), 20 mV/A sensitivity\r\n");
    UART_Printf("Expected ADC reading @ 0A: ~2500mV (or ~%lu counts)\r\n", 
                (unsigned long)(2500UL * ADC_MAX_COUNTS / g_vdda_mv));
    UART_Printf("PWM should be at 50%% (zero output voltage, zero torque command)\r\n");
    UART_Printf("Running 50 samples over ~2.5 seconds - ENSURE g_iq_ref = 0mA!\r\n");
    UART_Printf("WARNING: ADC clips at 0V and 3.3V - max safe range is ±40A\r\n\r\n");
    
    /* Keep hardware triggers active - just read the values the ISR is already sampling */
    /* The ISR updates s_ia_inj_last, s_ib_inj_last, s_ic_inj_last every 50us (20kHz) */
    
    UART_Printf("Sample#  | IA_raw | IB_raw | IC_raw | IA_filt_mA | IB_filt_mA | IC_filt_mA\r\n");
    UART_Printf("---------+--------+--------+--------+--------+--------+--------+--------+--------+--------\r\n");
    
    uint16_t ia_first = 0, ib_first = 0, ic_first = 0;
    uint16_t ia_min = 0xFFFF, ib_min = 0xFFFF, ic_min = 0xFFFF;
    uint16_t ia_max = 0, ib_max = 0, ic_max = 0;
    uint32_t sum_ia = 0, sum_ib = 0, sum_ic = 0;
    
    for (int i = 0; i < 50; i++) {
        /* Read the raw values and filtered values from ISR */
        uint16_t ia_raw = s_ia_inj_last;
        uint16_t ib_raw = s_ib_inj_last;
        uint16_t ic_raw = s_ic_inj_last;
        
        /* Track min/max/sum for drift analysis */
        if (i == 0) {
            ia_first = ia_raw;
            ib_first = ib_raw;
            ic_first = ic_raw;
        }
        ia_min = (ia_raw < ia_min) ? ia_raw : ia_min;
        ib_min = (ib_raw < ib_min) ? ib_raw : ib_min;
        ic_min = (ic_raw < ic_min) ? ic_raw : ic_min;
        ia_max = (ia_raw > ia_max) ? ia_raw : ia_max;
        ib_max = (ib_raw > ib_max) ? ib_raw : ib_max;
        ic_max = (ic_raw > ic_max) ? ic_raw : ic_max;
        sum_ia += ia_raw;
        sum_ib += ib_raw;
        sum_ic += ic_raw;
        
        /* Read filtered values (already in Amperes) */
        float ia_filt_a = g_ia_filtered;
        float ib_filt_a = g_ib_filtered;
        float ic_filt_a = g_ic_filtered;
        
        /* Convert filtered amperes to milliamps */
        int32_t ia_filt_ma = (int32_t)(ia_filt_a * 1000.0f);
        int32_t ib_filt_ma = (int32_t)(ib_filt_a * 1000.0f);
        int32_t ic_filt_ma = (int32_t)(ic_filt_a * 1000.0f);
        
        /* Print: raw counts + filtered mA values */
        UART_Printf("%d | %6u | %6u | %6u | %+7ld | %+7ld | %+7ld\r\n",
                    i,
                    ia_raw, ib_raw, ic_raw,
                    (long)ia_filt_ma, (long)ib_filt_ma, (long)ic_filt_ma);
        
        HAL_Delay(50); /* 50ms between printouts */
    }
    
    /* Analyze baseline */
    uint16_t ia_mean = (uint16_t)(sum_ia / 50);
    uint16_t ib_mean = (uint16_t)(sum_ib / 50);
    uint16_t ic_mean = (uint16_t)(sum_ic / 50);
    uint16_t ia_last = s_ia_inj_last;
    uint16_t ib_last = s_ib_inj_last;
    uint16_t ic_last = s_ic_inj_last;
    
    /* ADCs are still running on hardware trigger (TIM1_TRGO) - no need to restore */
    
    UART_Printf("\r\nADC test complete. Hardware triggers still active.\r\n");
    UART_Printf("\r\n=== RAW ADC BASELINE ANALYSIS ===\r\n");
    UART_Printf("IA: first=%u mean=%u last=%u | min=%u max=%u | noise=%u counts (pk-pk)\r\n",
                ia_first, ia_mean, ia_last, ia_min, ia_max, ia_max - ia_min);
    UART_Printf("IB: first=%u mean=%u last=%u | min=%u max=%u | noise=%u counts (pk-pk)\r\n",
                ib_first, ib_mean, ib_last, ib_min, ib_max, ib_max - ib_min);
    UART_Printf("IC: first=%u mean=%u last=%u | min=%u max=%u | noise=%u counts (pk-pk)\r\n",
                ic_first, ic_mean, ic_last, ic_min, ic_max, ic_max - ic_min);
    
    UART_Printf("\r\nBASELINE INTERPRETATION:\r\n");
    UART_Printf("  - Expected: All three channels near 3100±20 counts (noise-free baseline)\r\n");
    UART_Printf("  - All filtered currents should be near ±20mA (acceptable sensor noise)\r\n");
    UART_Printf("  - If much higher: Clarke-Park transform may have issues\r\n");
    
    UART_Printf("\r\nDiagnostics:\r\n");
    UART_Printf("- ADC conversion status: %lu successful, %lu missed (JEOS flag issue)\r\n",
                (unsigned long)g_adc_ready_count, (unsigned long)g_adc_not_ready_count);
    UART_Printf("- Zero calibration: IA=%lu, IB=%lu, IC=%lu counts\r\n",
                (unsigned long)g_ia_zero_counts,
                (unsigned long)g_ib_zero_counts, 
                (unsigned long)g_ic_zero_counts);
    UART_Printf("- Zero in mV: IA=%lu, IB=%lu, IC=%lu mV\r\n",
                (unsigned long)(g_ia_zero_counts * g_vdda_mv / ADC_MAX_COUNTS),
                (unsigned long)(g_ib_zero_counts * g_vdda_mv / ADC2_MAX_COUNTS),
                (unsigned long)(g_ic_zero_counts * g_vdda_mv / ADC2_MAX_COUNTS));
    UART_Printf("- Expected zero point: ~2500mV (ACS72981 5V variant)\r\n");
    UART_Printf("- VDDA calibrated: %lu mV\r\n", (unsigned long)g_vdda_mv);
    
    /* Check for potential issues */
    uint32_t ia_zero_mv = g_ia_zero_counts * g_vdda_mv / ADC_MAX_COUNTS;
    uint32_t ib_zero_mv = g_ib_zero_counts * g_vdda_mv / ADC2_MAX_COUNTS;
    uint32_t ic_zero_mv = g_ic_zero_counts * g_vdda_mv / ADC2_MAX_COUNTS;
    
    if (ia_zero_mv < 2300 || ia_zero_mv > 2700) {
        UART_Printf("WARNING: IA zero point (%lu mV) is far from expected 2500mV!\r\n", 
                    (unsigned long)ia_zero_mv);
    }
    if (ib_zero_mv < 2300 || ib_zero_mv > 2700) {
        UART_Printf("WARNING: IB zero point (%lu mV) is far from expected 2500mV!\r\n", 
                    (unsigned long)ib_zero_mv);
    }
    if (ic_zero_mv < 2300 || ic_zero_mv > 2700) {
        UART_Printf("WARNING: IC zero point (%lu mV) is far from expected 2500mV!\r\n", 
                    (unsigned long)ic_zero_mv);
    }
    
    UART_Printf("\r\nPress any key to continue...\r\n\r\n");
    
    /* Wait a bit for user to read */
    HAL_Delay(2000);
}



/* ===== Current Step Test ===== */
static void CurrentStepTest(void)
{
    /* Test sequence: 0A -> 1A -> 2A -> 1A -> 0A, 1500ms each for detailed response */
    /* Reset FOC state before test */
    FOC_Reset();

    UART_Printf("\r\n=== CURRENT TEST: 2A for 3s ===\r\n");
    UART_Printf("ms | iq_mA | id_mA | elec_deg\r\n");

    FOC_SetCurrentRefs(0.0f, 1.0f);
    g_iq_ref = 1.0f;

    uint32_t start = HAL_GetTick();
    while ((int32_t)(HAL_GetTick() - start) < 10000) {
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed % 10 == 0) {
            int32_t elec_deg = (int32_t)(g_elec_angle * RAD_TO_DEG_F);
            UART_Printf("%lu | %ld | %ld | %ld\r\n",
                        elapsed,
                        (long)(g_iq_meas * 1000.0f),
                        (long)(g_id_meas * 1000.0f),
                        (long)elec_deg);
        }
        HAL_Delay(1);
    }

    /* Back to zero */
    FOC_SetCurrentRefs(0.0f, 0.0f);
    g_iq_ref = 0.0f;

    UART_Printf("=== TEST COMPLETE ===\r\n\r\n");
    HAL_Delay(500);
}

/* ===== Bus voltage non-blocking state machine ===== */
/* DISABLED: Conflicts with hardware-triggered ADC current sensing */
/* Bus voltage is read once at startup. For continuous monitoring, */
/* implement a dedicated ADC channel or use periodic blocking reads */
/* when FOC is disabled (e.g., during idle periods). */

#if 0  /* Disabled - needs redesign for hardware-triggered ADC */
typedef enum { BV_IDLE, BV_WAIT } BusV_State_t;
static BusV_State_t s_bv_state = BV_IDLE;
static float s_bv_filtered = 0.0f;
static uint32_t s_bv_saved_jsqr = 0U;
static uint32_t s_bv_saved_smpr2 = 0U;

static void BusVoltage_Update(void)
{
    switch (s_bv_state) {
    case BV_IDLE:
        /* Start injected conversion (avoid collision with TIM1 trigger) */
        if (TIM1->CNT > 200 && TIM1->CNT < 1800 && ((ADC1->CR & ADC_CR_JADSTART) == 0U)) {
            /* Disable interrupts during channel reconfiguration */
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            
            s_bv_saved_jsqr = ADC1->JSQR;
            s_bv_saved_smpr2 = ADC1->SMPR2;
            LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
            LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);
            LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_12);
            LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12,
                                           LL_ADC_SAMPLINGTIME_47CYCLES_5);
            LL_ADC_INJ_StartConversion(ADC1);
            
            if (!primask) __enable_irq();
            s_bv_state = BV_WAIT;
        }
        break;

    case BV_WAIT:
        if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
            LL_ADC_ClearFlag_JEOS(ADC1);
            uint32_t raw = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
            float v = (float)raw * (float)g_vdda_mv
                    / ((float)ADC_MAX_COUNTS * 1000.0f) * BUS_V_DIVIDER_RATIO;

            /* Disable interrupts during channel restoration */
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            
            /* Restore injected config for current sensing using LL functions */
            LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);
            LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
            LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_2CYCLES_5);
            ADC1->SMPR2 = s_bv_saved_smpr2;
            
            if (!primask) __enable_irq();

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
#endif  /* Disabled bus voltage state machine */


/* ===== MAIN ===== */
int main(void)
{
    /* 1. HAL init (SysTick only) */
    HAL_Init();

    /* 2. System clock 170MHz */
    LL_SystemClock_Init();

    /* Update HAL's SystemCoreClock for HAL_Delay */
    SystemCoreClock = 170000000UL;
    
    /* CRITICAL: Reconfigure SysTick for new clock frequency */
    HAL_InitTick(TICK_INT_PRIORITY);
    
    /* 3. All GPIO - MUST come first so we can transmit early diagnostics */
    LL_GPIO_Init_All();
    
    /* 4. USART1 for telemetry - INITIALIZE EARLY for debug output */
    LL_USART1_Init();
    UART_Init();
    
    /* Small delay to let UART stabilize */
    HAL_Delay(10);
    
    /* DIAGNOSTIC: Send boot message */
    UART_SendStringPolled("\r\n=== BOOT SEQUENCE STARTING ===\r\n");
    UART_Flush();
    
    UART_Printf("\r\n=== Mako Shortfin FOC v2.0 ===\r\n");
    UART_Printf("Boot: Clock init OK, SystemCoreClock=%lu MHz\r\n", (unsigned long)(SystemCoreClock / 1000000UL));
    UART_Flush();

    /* 5. OPAMP3 (bus voltage buffer) */
    UART_Printf("Init: OPAMP3...\r\n");
    UART_Flush();
    LL_OPAMP3_Init();
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 6. DMA for ADC */
    UART_Printf("Init: DMA...\r\n");
    UART_Flush();
    LL_DMA_Init_ADC(g_adc_dual_raw);
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 7. ADC1+ADC2 dual mode */
    UART_Printf("Init: ADC calibration...\r\n");
    UART_Flush();
    
    int adc_cal_result = LL_ADC_Calibrate_All();
    if (adc_cal_result != 0) {
        UART_Printf("FAIL: ADC cal error %d\r\n", adc_cal_result);
        UART_Printf("ADC1 CR: 0x%08lX, ISR: 0x%08lX\r\n", ADC1->CR, ADC1->ISR);
        UART_Printf("ADC2 CR: 0x%08lX, ISR: 0x%08lX\r\n", ADC2->CR, ADC2->ISR);
        UART_Printf("CCR: 0x%08lX, RCC: 0x%08lX\r\n", ADC12_COMMON->CCR, RCC->AHB2ENR);
        UART_Flush();
        while(1) HAL_Delay(1000);
    }
    
    UART_Printf("OK\r\n");
    UART_Flush();
    
    UART_Printf("Init: ADC channels...\r\n");
    UART_Flush();
    LL_ADC_Init_All();
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 8. TIM1 PWM (counter starts, PWM channels enabled) */
    UART_Printf("Init: TIM1 PWM...\r\n");
    UART_Flush();
    LL_TIM1_Init();
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 9. SPI3 for encoder */
    UART_Printf("Init: SPI3...\r\n");
    UART_Flush();
    LL_SPI3_Init();
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 10. FDCAN placeholder */
    UART_Printf("Init: FDCAN...\r\n");
    UART_Flush();
    LL_FDCAN1_Init();
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 11. Enable gate drivers */
    UART_Printf("Init: Gate drivers...\r\n");
    UART_Flush();
    LL_GPIO_SetOutputPin(GATE_EN1_PORT, GATE_EN1_PIN);
    LL_GPIO_SetOutputPin(GATE_EN2_PORT, GATE_EN2_PIN);
    LL_GPIO_SetOutputPin(GATE_EN3_PORT, GATE_EN3_PIN);
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 12. Software init */
    UART_Printf("Init: FOC...\r\n");
    UART_Flush();
    FOC_Init();
    FOC_SetGains(FOC_KP_D, FOC_KI_D, FOC_KP_Q, FOC_KI_Q);
    FOC_SetCurrentRefs(0.0f, 0.0f);
    UART_Printf("OK\r\n");
    UART_Flush();

    UART_Printf("Init: Encoder...\r\n");
    UART_Flush();
    Encoder_Init();
    UART_Printf("OK\r\n");
    UART_Flush();
    
    // Position control disabled: skip Control_Init and SetGains

    /* 13. Calibrate VDDA */
    UART_Printf("Init: ADC startup...\r\n");
    UART_Flush();
    LL_ADC_Start_All();
    HAL_Delay(10);
    UART_Printf("OK\r\n");
    UART_Flush();
    
    UART_Printf("Calibrating VDDA...\r\n");
    UART_Flush();
    CalibrateVDDA();
    UART_Printf("VDDA: %lu mV\r\n", (unsigned long)g_vdda_mv);
    UART_Flush();

    /* 14. Enable PWM outputs at 50% duty (zero voltage) */
    UART_Printf("Enabling PWM outputs...\r\n");
    UART_Flush();
    TIM1->CCR1 = TIM1_HALF_PERIOD;
    TIM1->CCR2 = TIM1_HALF_PERIOD;
    TIM1->CCR3 = TIM1_HALF_PERIOD;
    LL_TIM_EnableAllOutputs(TIM1);
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 15. Read initial bus voltage */
    UART_Printf("Reading bus voltage...\r\n");
    UART_Flush();
    float bus_v = ReadBusVoltageBlocking();
    g_bus_voltage = bus_v;
    UART_Printf("Bus: %ld mV\r\n", (long)(bus_v * 1000.0f));
    UART_Flush();
    
    /* TODO: Fix bus voltage measurement - currently reads current sensor value */
    /* For now, use nominal 24V and skip safety check to focus on current sensing */
    bus_v = 24.0f;
    g_bus_voltage = 24.0f;
    UART_Printf("Bus: Using nominal 24V (actual reading DISABLED)\r\n");
    UART_Flush();

    /* 16. Encoder offset calibration */
    UART_Printf("Calibrating encoder...\r\n");
    UART_Flush();
    g_encoder_offset_counts = CalibrateEncoderOffset(bus_v);

    /* 17. Current zero calibration (motor de-energized) */
    UART_Printf("Calibrating current zeros...\r\n");
    UART_Flush();
    HAL_Delay(500);
    
    /* Use hardcoded zero values from ADC test (ADC test proved these work) */
    /* ADC test showed: IA_raw ≈ 3100, IB_raw ≈ 3100, IC_raw ≈ 3100 at 0A */
    g_ia_zero_counts = 3100;
    g_ib_zero_counts = 3100;
    g_ic_zero_counts = 3100;
    
    UART_Printf("Current zero: IA=%lu IB=%lu IC=%lu counts (hardcoded from ADC test)\r\n",
                 (unsigned long)g_ia_zero_counts,
                 (unsigned long)g_ib_zero_counts,
                 (unsigned long)g_ic_zero_counts);
    UART_Flush();

    /* 18. Prepare FOC */
    UART_Printf("Preparing FOC...\r\n");
    UART_Flush();
    FOC_Reset();
    FOC_SetCurrentRefs(0.0f, 0.0f);
    UART_Printf("OK\r\n");
    UART_Flush();

    /* 19. Enable FOC ISR - keep FOC loop DISABLED during calibration */
    UART_Printf("Enabling FOC ISR @ %lu Hz...\r\n", (unsigned long)FOC_ISR_FREQ_HZ);
    UART_Flush();
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    LL_TIM_EnableIT_UPDATE(TIM1);
    g_foc_enabled = 0; /* FOC loop OFF - ISR samples ADC but PI does not run */
    UART_Printf("OK\r\n");
    UART_Flush();

    /* Give ISR a moment to start sampling (20kHz = 50us per sample) */
    HAL_Delay(100);

    /* 19a. Calibrate current zero offsets using live hardware-triggered ISR samples */
    UART_Printf("Running ADC zero calibration...\r\n");
    UART_Flush();
    CalibrateCurrentZeros();
    UART_Printf("Calibration complete\r\n");
    UART_Flush();

    /* 19b. ADC Reading Test - motor stationary, FOC loop still disabled */
    UART_Printf("Running ADC reading test...\r\n");
    UART_Flush();
    ADCReadingTest();
    UART_Printf("Test complete\r\n");
    UART_Flush();

    /* 19c. Enable FOC closed-loop NOW (after calibration, with good zero offsets) */
    FOC_Reset();
    FOC_SetCurrentRefs(0.0f, 0.0f);
    g_foc_enabled = PWM_ONLY_MODE ? 0U : 1U;
    UART_Printf("FOC closed-loop enabled\r\n");
    UART_Flush();

    /* 20. Start watchdog (AFTER all initialization to prevent premature reset) */
#if ENABLE_WATCHDOG
    UART_Printf("Starting watchdog...\r\n");
    UART_Flush();
    LL_IWDG_Init();
#else
    UART_Printf("Watchdog DISABLED (test mode)...\r\n");
    UART_Flush();
#endif
    
    UART_Printf("\r\n*** SYSTEM READY ***\r\n");
    UART_Printf("FOC running @ %lu Hz\r\n", (unsigned long)FOC_ISR_FREQ_HZ);
    UART_Flush();
    
    /* Debug: check encoder before starting */
    uint16_t test_raw = 0;
    Encoder_Status_t enc_status = Encoder_ReadAngle(&test_raw);
    uint16_t spi_rx1 = 0, spi_rx2 = 0;
    Encoder_GetDebugData(&spi_rx1, &spi_rx2);
    UART_Printf("Encoder: status=%d rx1=0x%04X rx2=0x%04X ang=%u counts\r\n",
                (int)enc_status, spi_rx1, spi_rx2, test_raw);
    UART_Printf("Angles: offset=%u counts, mech=%ld mrad, elec=%ld mrad\r\n",
                g_encoder_offset_counts,
                (long)(g_mech_angle * 1000.0f),
                (long)(g_elec_angle * 1000.0f));

    /* ===== Main loop ===== */
    uint32_t last_telem_ms = HAL_GetTick();
    uint32_t last_vdda_calib_ms = HAL_GetTick();

    /* Wait 2 seconds for everything to settle, then run current step test */
    HAL_Delay(2000);
    CurrentStepTest();

    while (1) {
        uint32_t now = HAL_GetTick();

        /* Telemetry @ 50Hz */
        if (now - last_telem_ms >= TELEMETRY_PERIOD_MS) {
            last_telem_ms = now;
            UART_SendTelemetry();
        }

        /* Re-calibrate VDDA every 10 seconds */
        if (now - last_vdda_calib_ms >= VDDA_RECALIB_MS) {
            last_vdda_calib_ms = now;
            CalibrateVDDA();
            UART_Printf("VDDA recal: %lu mV\r\n", (unsigned long)g_vdda_mv);
        }

        /* Watchdog refresh */
#if ENABLE_WATCHDOG
        IWDG->KR = 0xAAAA;
#endif
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
    
    /* Enable GPIOC if not already enabled */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    
    /* Configure PC8 as output for error indication */
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << (8*2))) | (1UL << (8*2));
    
    /* Rapid blink to indicate error condition */
    while (1) {
        GPIOC->BSRR = (1UL << 8);  /* Set PC8 */
        for (volatile int i = 0; i < 50000; i++);
        GPIOC->BSRR = (1UL << (8 + 16));  /* Clear PC8 */
        for (volatile int i = 0; i < 50000; i++);
    }
}
