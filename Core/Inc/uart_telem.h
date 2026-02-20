#ifndef UART_TELEM_H
#define UART_TELEM_H

#include <stdint.h>

void UART_Init(void);
void UART_Printf(const char *fmt, ...);
void UART_SendTelemetry(void);
void UART_IRQHandler(void);

/* Direct polled transmit for diagnostics */
void UART_SendCharPolled(char c);
void UART_SendStringPolled(const char *str);

/* Flush any pending TX buffer data (wait for ISR to complete) */
void UART_Flush(void);

#endif /* UART_TELEM_H */
