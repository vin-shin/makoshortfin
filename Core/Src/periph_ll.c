#include "periph_ll.h"
#include "config.h"

/* ===== System Clock: HSI 16MHz -> PLL -> 170MHz ===== */
void LL_SystemClock_Init(void)
{
    /* Enable power interface clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* Voltage scaling 1 boost for 170MHz */
    LL_PWR_EnableRange1BoostMode();

    /* Flash: 4 wait states for 170MHz, prefetch + caches */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {}
    LL_FLASH_EnableInstCache();
    LL_FLASH_EnableDataCache();

    /* Enable HSI with timeout */
    LL_RCC_HSI_Enable();
    uint32_t hsi_timeout = 5000;
    while (!LL_RCC_HSI_IsReady()) {
        if (--hsi_timeout == 0) {
            /* HSI failed to start - use Error_Handler */
            extern void Error_Handler(void);
            Error_Handler();
        }
    }

    /* PLL: HSI(16MHz) / 4 * 85 / 2 = 170MHz */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85,
                                 LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    
    uint32_t pll_timeout = 5000;
    while (!LL_RCC_PLL_IsReady()) {
        if (--pll_timeout == 0) {
            /* PLL failed to lock */
            extern void Error_Handler(void);
            Error_Handler();
        }
    }

    /* System clock from PLL with timeout */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    uint32_t sysclk_timeout = 5000;
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
        if (--sysclk_timeout == 0) {
            /* Clock switch failed */
            extern void Error_Handler(void);
            Error_Handler();
        }
    }

    /* AHB, APB1, APB2 all at 170MHz (no division) */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Update SystemCoreClock variable */
    SystemCoreClock = 170000000UL;
}

/* ===== GPIO helper: direct register config for one pin ===== */
/* mode: 0=input, 1=output, 2=alternate, 3=analog                */
/* otype: 0=push-pull, 1=open-drain                               */
/* speed: 0=low, 1=medium, 2=high, 3=very_high                   */
/* pull: 0=none, 1=pull-up, 2=pull-down                           */
/* af: 0-15 alternate function (only used when mode==2)           */
static void gpio_pin_config(GPIO_TypeDef *port, uint32_t pin,
                             uint32_t mode, uint32_t otype,
                             uint32_t speed, uint32_t pull, uint32_t af)
{
    uint32_t pos2 = pin * 2U;

    /* MODER: 2 bits per pin */
    port->MODER  = (port->MODER  & ~(3UL << pos2)) | (mode  << pos2);
    /* OTYPER: 1 bit per pin */
    port->OTYPER = (port->OTYPER & ~(1UL << pin))  | (otype << pin);
    /* OSPEEDR: 2 bits per pin */
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << pos2)) | (speed << pos2);
    /* PUPDR: 2 bits per pin */
    port->PUPDR  = (port->PUPDR  & ~(3UL << pos2)) | (pull  << pos2);
    /* AFR: 4 bits per pin; AFR[0] for pins 0-7, AFR[1] for pins 8-15 */
    if (mode == 2U) {
        uint32_t idx = pin >> 3U;           /* 0 or 1 */
        uint32_t shift = (pin & 7U) * 4U;
        port->AFR[idx] = (port->AFR[idx] & ~(0xFUL << shift)) | (af << shift);
    }
}

/* ===== GPIO: All pin configuration ===== */
void LL_GPIO_Init_All(void)
{
    /* Enable all GPIO clocks */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);

    /* --- TIM1 PWM: PA8,PA9,PA10 (CH1-3), PA11,PA12 (CH1N,CH2N) AF6 --- */
    gpio_pin_config(GPIOA,  8, 2, 0, 3, 0, 6);  /* PA8  TIM1_CH1  */
    gpio_pin_config(GPIOA,  9, 2, 0, 3, 0, 6);  /* PA9  TIM1_CH2  */
    gpio_pin_config(GPIOA, 10, 2, 0, 3, 0, 6);  /* PA10 TIM1_CH3  */
    gpio_pin_config(GPIOA, 11, 2, 0, 3, 0, 6);  /* PA11 TIM1_CH1N */
    gpio_pin_config(GPIOA, 12, 2, 0, 3, 0, 6);  /* PA12 TIM1_CH2N */
    /* PB15: TIM1_CH3N AF4 on G474 */
    gpio_pin_config(GPIOB, 15, 2, 0, 3, 0, 4);  /* PB15 TIM1_CH3N */

    /* --- ADC analog inputs: PA0 (ADC1_IN1), PA6 (ADC2_IN3), PA7 (ADC2_IN4) --- */
    gpio_pin_config(GPIOA, 0, 3, 0, 0, 0, 0);   /* PA0 analog */
    gpio_pin_config(GPIOA, 6, 3, 0, 0, 0, 0);   /* PA6 analog */
    gpio_pin_config(GPIOA, 7, 3, 0, 0, 0, 0);   /* PA7 analog */

    /* --- OPAMP3: PB0 (VINP), PB1 (VOUT) analog --- */
    gpio_pin_config(GPIOB, 0, 3, 0, 0, 0, 0);   /* PB0 analog */
    gpio_pin_config(GPIOB, 1, 3, 0, 0, 0, 0);   /* PB1 analog */

    /* --- SPI3: PC10 (SCK), PC11 (MISO), PC12 (MOSI) AF6 --- */
    gpio_pin_config(GPIOC, 10, 2, 0, 3, 0, 6);  /* PC10 SPI3_SCK  */
    gpio_pin_config(GPIOC, 11, 2, 0, 3, 0, 6);  /* PC11 SPI3_MISO */
    gpio_pin_config(GPIOC, 12, 2, 0, 3, 0, 6);  /* PC12 SPI3_MOSI */

    /* --- SPI3 CS: PA15 GPIO output, default high --- */
    GPIOA->BSRR = (1UL << 15);                   /* Set high before switching to output */
    gpio_pin_config(GPIOA, 15, 1, 0, 3, 1, 0);  /* PA15 output, pull-up */

    /* --- USART1 TX: PB6 AF7 --- */
    gpio_pin_config(GPIOB, 6, 2, 0, 2, 0, 7);   /* PB6 USART1_TX */

    /* --- Gate driver enables: PC7, PC8, PC9 push-pull, default low --- */
    GPIOC->BRR = (1UL << 7) | (1UL << 8) | (1UL << 9);  /* Clear outputs first */
    gpio_pin_config(GPIOC, 7, 1, 0, 0, 0, 0);   /* PC7 gate enable */
    gpio_pin_config(GPIOC, 8, 1, 0, 0, 0, 0);   /* PC8 gate enable */
    gpio_pin_config(GPIOC, 9, 1, 0, 0, 0, 0);   /* PC9 gate enable */

    /* --- FDCAN1 placeholder: PB8 (RX), PB9 (TX) AF9 --- */
    gpio_pin_config(GPIOB, 8, 2, 0, 2, 0, 9);   /* PB8 FDCAN1_RX */
    gpio_pin_config(GPIOB, 9, 2, 0, 2, 0, 9);   /* PB9 FDCAN1_TX */
}

/* ===== TIM1: Center-aligned PWM + ADC trigger ===== */
void LL_TIM1_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* Base: center-aligned mode 1, ARR=2125 -> 40kHz triangle */
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_CENTER_UP);
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetAutoReload(TIM1, TIM1_ARR_VALUE);
    LL_TIM_SetRepetitionCounter(TIM1, TIM1_RCR_VALUE);
    LL_TIM_EnableARRPreload(TIM1);

    /* CH1: PWM1, preload */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH1(TIM1, TIM1_HALF_PERIOD);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1N, LL_TIM_OCPOLARITY_HIGH);

    /* CH2: PWM1, preload */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH2(TIM1, TIM1_HALF_PERIOD);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCPOLARITY_HIGH);

    /* CH3: PWM1, preload */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH3(TIM1, TIM1_HALF_PERIOD);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3N, LL_TIM_OCPOLARITY_HIGH);

    /* CH4: ADC trigger point - set slightly before center for ADC completion before ISR */
    /* PWM center is at TIM1_HALF_PERIOD (1062). Trigger ADC ~50 counts early (~294ns @ 170MHz) */
    /* This gives ADC time to complete 3-channel conversion (~2us) before ISR fires */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH4(TIM1, TIM1_HALF_PERIOD - 350U); /* Trigger ~2us early */

    /* TRGO = OC4REF -> triggers injected ADC (current sensing) early */
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
    /* TRGO2 = Update event with RCR -> triggers ISR after ADC completes */
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_UPDATE);

    /* Dead-time configuration (direct register write) */
    /* DTG=38 (~223ns), no lock, no OSSI/OSSR, break disabled, AOE disabled */
    TIM1->BDTR = (TIM1_DEADTIME << TIM_BDTR_DTG_Pos);

    /* Enable channels (but don't enable MOE yet - done in main after calibration) */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N
                                | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N
                                | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N
                                | LL_TIM_CHANNEL_CH4);

    /* Start counter */
    LL_TIM_EnableCounter(TIM1);

    /* Force update to load shadow registers */
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_ClearFlag_UPDATE(TIM1);
}

/* ===== DMA1 Channel 1: ADC dual-mode circular ===== */
void LL_DMA_Init_ADC(volatile uint32_t *dest)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

    /* DMA1 Channel 1 for ADC1 dual-mode */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2); /* 2 ranks: both ADCs convert twice */

    /* Addresses: source = ADC12_COMMON->CDR, dest = g_adc_dual_raw */
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                           (uint32_t)&(ADC12_COMMON->CDR),
                           (uint32_t)dest,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

/* ===== ADC1 + ADC2: Dual-mode simultaneous ===== */
void LL_ADC_Init_All(void)
{
    /* ADC clock was already configured in LL_ADC_Calibrate_All() */
    /* This function now only configures channels and sequencers */
    
    /* Ensure both ADCs are disabled before configuring common registers */
    if (LL_ADC_IsEnabled(ADC1) || LL_ADC_IsEnabled(ADC2)) {
        /* This should not happen but guard against it */
        if (LL_ADC_IsEnabled(ADC1)) LL_ADC_Disable(ADC1);
        if (LL_ADC_IsEnabled(ADC2)) LL_ADC_Disable(ADC2);
        volatile uint32_t wait = 10000;
        while (wait--) {}
    }
    
    /* Configure ADC common: dual regular simultaneous mode */
    /* Use LL functions to be safe */
    LL_ADC_SetMultimode(ADC12_COMMON, LL_ADC_MULTI_DUAL_REG_SIMULT);
    LL_ADC_SetMultiDMATransfer(ADC12_COMMON, LL_ADC_MULTI_REG_DMA_UNLMT_RES12_10B);

    /* Resolution, data alignment */
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);

    /* Oversampling: 16x, right shift 4, apply to injected group */
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_INJECTED);
    LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_16,
                                         LL_ADC_OVS_SHIFT_RIGHT_4);

    /* Regular channel config: 2 conversions, TIM1_TRGO2 trigger */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
    LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,
                                   LL_ADC_SAMPLINGTIME_12CYCLES_5);

    /* Injected: phase A current on TIM1_TRGO (=OC4REF, fires ~2us before ISR) */
    LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_SetSequencerLength(ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE);
    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,
                                   LL_ADC_SAMPLINGTIME_12CYCLES_5);

    /* === ADC2 === */
    LL_ADC_SetResolution(ADC2, LL_ADC_RESOLUTION_12B);

    /* Oversampling: 16x, right shift 4, apply to injected group */
    LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_GRP_INJECTED);
    LL_ADC_ConfigOverSamplingRatioShift(ADC2, LL_ADC_OVS_RATIO_16,
                                         LL_ADC_OVS_SHIFT_RIGHT_4);

    /* ADC2 regular: CH3 (PA6), CH4 (PA7), slave in dual mode */
    /* CRITICAL: In STM32G4 dual mode, slave ADC must have SAME trigger config as master! */
    LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
    LL_ADC_REG_SetTriggerEdge(ADC2, LL_ADC_REG_TRIG_EXT_RISING);
    LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_NONE); /* Multi-DMA via CCR */
    LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_4);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3,
                                   LL_ADC_SAMPLINGTIME_12CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4,
                                   LL_ADC_SAMPLINGTIME_12CYCLES_5);

    /* Injected: phase B/C currents on TIM1_TRGO (=OC4REF, fires ~2us before ISR) */
    LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO);
    LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);
    LL_ADC_INJ_SetSequencerLength(ADC2, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_3);
    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_4);
}

/* Calibrate and enable both ADCs - returns 0 on success, non-zero on error */
int LL_ADC_Calibrate_All(void)
{
    /* Force ADC clock enable */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    
    /* Hardware reset ADC peripherals */
    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADC12RST;
    volatile uint32_t dly = 100;
    while (dly--) {}
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADC12RST;
    
    /* Wait after reset */
    dly = 1000;
    while (dly--) {}
    
    /* CRITICAL: Configure ADC clock AFTER reset, BEFORE any other config */
    /* Use synchronous clock mode: HCLK/4 = 170MHz/4 = 42.5MHz */
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_CKMODE) | (0x03UL << ADC_CCR_CKMODE_Pos);
    
    /* Wait for clock to stabilize */
    dly = 1000;
    while (dly--) {}
    
    /* Ensure ADCs are disabled before calibration */
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
        uint32_t timeout = 10000;
        while (LL_ADC_IsDisableOngoing(ADC1) && --timeout);
        if (timeout == 0) return 1;
    }
    
    if (LL_ADC_IsEnabled(ADC2)) {
        LL_ADC_Disable(ADC2);
        uint32_t timeout = 10000;
        while (LL_ADC_IsDisableOngoing(ADC2) && --timeout);
        if (timeout == 0) return 2;
    }

    /* Exit deep power-down mode if needed */
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_DisableDeepPowerDown(ADC2);
    
    /* Small delay after exiting deep power-down */
    dly = 1000;
    while (dly--) {}

    /* Enable internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    LL_ADC_EnableInternalRegulator(ADC2);

    /* Wait for regulator startup (20us min per datasheet, use 100us to be very safe) */
    volatile uint32_t wait = 170 * 100; /* ~100us at 170MHz */
    while (wait--) {}

    /* Calibrate single-ended with timeout */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    
    /* Check if ADCAL bit was actually set */
    if (!(ADC1->CR & ADC_CR_ADCAL)) {
        return 20; /* ADC1 calibration did not start */
    }
    
    uint32_t adc1_cal_timeout = 500000; /* ~3ms timeout */
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {
        if (--adc1_cal_timeout == 0) {
            return 3; /* ADC1 calibration timeout */
        }
    }

    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    
    /* Check if ADCAL bit was actually set */
    if (!(ADC2->CR & ADC_CR_ADCAL)) {
        return 21; /* ADC2 calibration did not start */
    }
    
    uint32_t adc2_cal_timeout = 500000; /* ~3ms timeout */
    while (LL_ADC_IsCalibrationOnGoing(ADC2)) {
        if (--adc2_cal_timeout == 0) {
            return 4; /* ADC2 calibration timeout */
        }
    }
    
    return 0; /* Success */
}

void LL_ADC_Start_All(void)
{
    /* Enable ADCs with timeout */
    LL_ADC_Enable(ADC1);
    uint32_t adc1_rdy_timeout = 50000;
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {
        if (--adc1_rdy_timeout == 0) {
            extern void Error_Handler(void);
            Error_Handler();
        }
    }
    LL_ADC_ClearFlag_ADRDY(ADC1);

    LL_ADC_Enable(ADC2);
    uint32_t adc2_rdy_timeout = 50000;
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2)) {
        if (--adc2_rdy_timeout == 0) {
            extern void Error_Handler(void);
            Error_Handler();
        }
    }
    LL_ADC_ClearFlag_ADRDY(ADC2);

    /* Enable conversion pipelines to respond to their configured trigger sources.
     * The trigger source is set to TIM1_TRGO in LL_ADC_Init_All(), so these calls
     * enable the injected conversions to fire on each TIM1_TRGO rising edge.
     * This is NOT software triggering - it's just arming the pipeline.
     */
    LL_ADC_INJ_StartConversion(ADC1);  /* Injected conversions (current sensing) */
    LL_ADC_INJ_StartConversion(ADC2);  /* Injected conversions (current sensing) */
    LL_ADC_REG_StartConversion(ADC1);  /* Regular conversions (if used) */
}

/* ===== SPI3: A1333 encoder, Mode 3, 16-bit ===== */
void LL_SPI3_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

    LL_SPI_Disable(SPI3);

    LL_SPI_SetMode(SPI3, LL_SPI_MODE_MASTER);
    LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_SetClockPolarity(SPI3, LL_SPI_POLARITY_HIGH);     /* CPOL=1 */
    LL_SPI_SetClockPhase(SPI3, LL_SPI_PHASE_2EDGE);          /* CPHA=1 */
    LL_SPI_SetBaudRatePrescaler(SPI3, LL_SPI_BAUDRATEPRESCALER_DIV32); /* 5.3MHz */
    LL_SPI_SetTransferDirection(SPI3, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetDataWidth(SPI3, LL_SPI_DATAWIDTH_16BIT);
    LL_SPI_SetNSSMode(SPI3, LL_SPI_NSS_SOFT);
    LL_SPI_SetTransferBitOrder(SPI3, LL_SPI_MSB_FIRST);
    LL_SPI_SetRxFIFOThreshold(SPI3, LL_SPI_RX_FIFO_TH_HALF); /* 16-bit threshold */

    LL_SPI_Enable(SPI3);
}

/* ===== USART1: TX only, 115200 8N1 ===== */
void LL_USART1_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_USART_Disable(USART1);

    LL_USART_SetBaudRate(USART1, 170000000UL, LL_USART_PRESCALER_DIV1,
                          LL_USART_OVERSAMPLING_16, UART_BAUD);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);

    LL_USART_Enable(USART1);

    /* Wait for USART ready (TX enabled) */
    volatile uint32_t timeout = 10000;
    while (!LL_USART_IsActiveFlag_TEACK(USART1) && --timeout) {}
    
    /* Small delay for stability */
    for (volatile int i = 0; i < 1000; i++);

    /* Enable TXE interrupt for async TX */
    LL_USART_EnableIT_TXE(USART1);
}

/* ===== OPAMP3: Follower for bus voltage ===== */
void LL_OPAMP3_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    /* Configure OPAMP3 in follower mode:
     * VPSEL = 00 (VINP0 = PB0)
     * VMSEL = 11 (output connected to VINM = follower mode)
     * OPAMPINTEN = 1 (internal output routed to ADC)
     * OPAMPxEN = 1 (enable) */
    OPAMP3->CSR = OPAMP_CSR_VMSEL          /* VMSEL = 11 -> follower */
                | OPAMP_CSR_OPAMPINTEN     /* Internal output to ADC */
                | OPAMP_CSR_OPAMPxEN;      /* Enable */
}

/* ===== FDCAN1: Placeholder init ===== */
void LL_FDCAN1_Init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
    /* Placeholder - no further config needed until CAN is used */
}

/* ===== IWDG: ~1s timeout ===== */
void LL_IWDG_Init(void)
{
    /* Unlock and configure */
    IWDG->KR = 0x5555;                   /* Enable write access */
    IWDG->PR = 4;                         /* Prescaler /64 -> 32kHz/64 = 500Hz */
    IWDG->RLR = 500;                      /* Reload = 500 -> 1s timeout */
    while (IWDG->SR) {}                   /* Wait for registers to update */
    IWDG->KR = 0xCCCC;                   /* Start watchdog */
    IWDG->KR = 0xAAAA;                   /* Reload */
}
