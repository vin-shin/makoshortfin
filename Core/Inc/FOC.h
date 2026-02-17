#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <math.h>
#include "config.h"

typedef struct {
    float ia;
    float ib;
    float ic;
    float bus_v;
    float elec_angle;
} FOC_Sensors_t;

typedef struct {
    float id_meas;
    float iq_meas;
    float vd;
    float vq;
    float duty_u;
    float duty_v;
    float duty_w;
} FOC_Output_t;

void FOC_Init(void);
void FOC_Reset(void);
void FOC_SetCurrentRefs(float id_ref, float iq_ref);
void FOC_SetGains(float kp_d, float ki_d, float kp_q, float ki_q);
void FOC_Run(const FOC_Sensors_t *sensors, FOC_Output_t *output);
void FOC_SinCos(float angle_rad, float *sin_out, float *cos_out);
static inline float FOC_WrapAngle2Pi(float angle_rad)
{
    if (angle_rad >= TWO_PI_F) {
        angle_rad -= TWO_PI_F;
    } else if (angle_rad < 0.0f) {
        angle_rad += TWO_PI_F;
    }
    return angle_rad;
}

static inline float FOC_WrapAnglePi(float angle_rad)
{
    angle_rad += PI_F;
    angle_rad = FOC_WrapAngle2Pi(angle_rad);
    return angle_rad - PI_F;
}

#endif /* FOC_H */
