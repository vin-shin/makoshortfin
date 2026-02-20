#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
    ENC_OK = 0,
    ENC_ERR_SPI,
} Encoder_Status_t;

void Encoder_Init(void);
/* Returns raw 15-bit mechanical count [0, 32767] — no float conversion */
Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts);
void Encoder_GetDebugData(uint16_t *rx1, uint16_t *rx2);

#endif /* ENCODER_H */
