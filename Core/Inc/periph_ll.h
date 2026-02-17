#ifndef PERIPH_LL_H
#define PERIPH_LL_H

#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_dmamux.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_utils.h"

void LL_SystemClock_Init(void);
void LL_GPIO_Init_All(void);
void LL_TIM1_Init(void);
void LL_ADC_Init_All(void);
void LL_DMA_Init_ADC(volatile uint32_t *dest);
void LL_SPI3_Init(void);
void LL_USART1_Init(void);
void LL_OPAMP3_Init(void);
void LL_FDCAN1_Init(void);
void LL_IWDG_Init(void);

void LL_ADC_Calibrate_All(void);
void LL_ADC_Start_All(void);

#endif /* PERIPH_LL_H */
