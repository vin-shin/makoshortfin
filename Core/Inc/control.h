#ifndef CONTROL_H
#define CONTROL_H

void Control_Init(void);
void Control_Reset(void);
void Control_SetTarget(float pos_rad);
void Control_SetGains(float kp, float ki, float kd);
float Control_Update(float pos_meas_rad, float dt_sec);

#endif /* CONTROL_H */
