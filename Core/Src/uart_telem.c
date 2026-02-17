#include "uart_telem.h"
#include "config.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Circular TX buffer */
static char s_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0;
static volatile uint16_t s_tx_tail = 0;

/* Shared telemetry state (written by ISR, read here) */
extern volatile float g_id_meas;
extern volatile float g_iq_meas;
extern volatile float g_vd;
extern volatile float g_vq;
extern volatile float g_bus_voltage;
extern volatile float g_mech_angle;
extern volatile float g_iq_ref;
extern volatile uint32_t g_isr_max_cycles;
extern volatile uint8_t g_fault_code;

/* Position target (written by main loop) */
extern float g_pos_target;

void UART_Init(void)
{
    /* USART1 already initialized by LL_USART1_Init() */
    /* Enable TXE interrupt for async TX */
    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);
}

/* Called from USART1_IRQHandler in stm32g4xx_it.c */
void UART_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_TXE(USART1) && LL_USART_IsEnabledIT_TXE(USART1)) {
        if (s_tx_tail != s_tx_head) {
            LL_USART_TransmitData8(USART1, (uint8_t)s_tx_buf[s_tx_tail]);
            s_tx_tail = (s_tx_tail + 1) % UART_TX_BUF_SIZE;
        } else {
            LL_USART_DisableIT_TXE(USART1);
        }
    }
}

void UART_Printf(const char *fmt, ...)
{
    char local[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(local, sizeof(local), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > (int)sizeof(local)) len = (int)sizeof(local);

    for (int i = 0; i < len; i++) {
        uint16_t next = (s_tx_head + 1) % UART_TX_BUF_SIZE;
        if (next == s_tx_tail) break;  /* Buffer full, drop */
        s_tx_buf[s_tx_head] = local[i];
        s_tx_head = next;
    }

    LL_USART_EnableIT_TXE(USART1);
}

void UART_SendTelemetry(void)
{
    static const char *fault_names[] = {"OK", "OC", "OV", "UV", "KILL"};

    int32_t pos_deg = (int32_t)(g_mech_angle * RAD_TO_DEG_F);
    int32_t tgt_deg = (int32_t)(g_pos_target * RAD_TO_DEG_F);
    int32_t id_ma = (int32_t)(g_id_meas * 1000.0f);
    int32_t iq_ma = (int32_t)(g_iq_meas * 1000.0f);
    int32_t iq_ref_ma = (int32_t)(g_iq_ref * 1000.0f);
    int32_t bus_mv = (int32_t)(g_bus_voltage * 1000.0f);
    uint32_t isr_us = g_isr_max_cycles / 170U;

    uint8_t fc = g_fault_code;
    if (fc > 0) {
        const char *name = (fc < 5) ? fault_names[fc] : "UNK";
        UART_Printf("FAULT:%s bus:%ld.%01ldV id:%ldmA iq:%ldmA\r\n",
                     name,
                     (long)(bus_mv / 1000), (long)((bus_mv % 1000) / 100),
                     (long)id_ma, (long)iq_ma);
    } else {
        UART_Printf("pos:%ld>%lddeg ref:%ldmA iq:%ldmA id:%ldmA bus:%ld.%01ldV %luus\r\n",
                     (long)pos_deg, (long)tgt_deg,
                     (long)iq_ref_ma, (long)iq_ma, (long)id_ma,
                     (long)(bus_mv / 1000), (long)((bus_mv % 1000) / 100),
                     (unsigned long)isr_us);
    }

    g_isr_max_cycles = 0;
}
