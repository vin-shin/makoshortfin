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
    float omega_e;          /* Electrical angular velocity (rad/s) for dq decoupling */
    uint16_t elec_counts;   /* 15-bit electrical angle [0, 32767] — no radians needed */
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

/* CORDIC sin/cos directly from 15-bit count [0, 32767].
 * Conversion: q31 = (counts - 16384) << 17  — one subtract, one shift, no float. */
void FOC_SinCos(uint16_t angle_counts, float *sin_out, float *cos_out);

#endif /* FOC_H */
