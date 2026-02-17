#ifndef FOC_H
#define FOC_H

#include <stdint.h>

typedef struct {
    float ia;
    float ib;
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

#endif /* FOC_H */
