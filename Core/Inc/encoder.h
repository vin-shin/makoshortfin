#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
    ENC_OK = 0,
    ENC_ERR_SPI,
} Encoder_Status_t;

void Encoder_Init(void);
Encoder_Status_t Encoder_ReadAngle(float *mech_angle_rad);
void Encoder_SetOffset(float offset_rad);
float Encoder_GetOffset(void);

#endif /* ENCODER_H */
