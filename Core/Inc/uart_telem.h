#ifndef UART_TELEM_H
#define UART_TELEM_H

#include <stdint.h>

void UART_Init(void);
void UART_Printf(const char *fmt, ...);
void UART_SendTelemetry(void);
void UART_IRQHandler(void);

#endif /* UART_TELEM_H */
