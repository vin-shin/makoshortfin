#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  float ia;
  float ib;
  float ic;
  float bus_v;
  float electrical_angle;
} FOC_SensorData;

typedef struct
{
  float id_ref;
  float iq_ref;
  float vd;
  float vq;
  float duty_u;
  float duty_v;
  float duty_w;
} FOC_Output;

void FOC_Init(void);
void FOC_Reset(void);
void FOC_SetPolePairs(uint16_t pole_pairs);
void FOC_SetCurrentOffsets(float ia_offset, float ib_offset, float ic_offset);
void FOC_SetCurrentRefs(float id_ref, float iq_ref);
void FOC_SetCurrentGains(float kp_d, float ki_d, float kp_q, float ki_q);
void FOC_SetControlPeriod(float dt_seconds);
void FOC_UpdateSensors(const FOC_SensorData *data);
void FOC_Run(FOC_Output *out);
void FOC_SinCos(float angle, float *sin_out, float *cos_out);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
