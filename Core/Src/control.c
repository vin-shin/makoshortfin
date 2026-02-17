#include "control.h"
#include "config.h"

static float s_target = 0.0f;
static float s_last_error = 0.0f;
static float s_integrator = 0.0f;
static float s_deriv_filt = 0.0f;

static float s_kp = POS_KP;
static float s_ki = POS_KI;
static float s_kd = POS_KD;

void Control_Init(void)
{
    Control_Reset();
}

void Control_Reset(void)
{
    s_last_error = 0.0f;
    s_integrator = 0.0f;
    s_deriv_filt = 0.0f;
}

void Control_SetTarget(float pos_rad)
{
    s_target = pos_rad;
}

void Control_SetGains(float kp, float ki, float kd)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
}

float Control_Update(float pos_meas_rad, float dt_sec)
{
    /* Shortest-path error with wraparound */
    float error = s_target - pos_meas_rad;
    if (error > PI_F) error -= TWO_PI_F;
    if (error < -PI_F) error += TWO_PI_F;

    /* Integrator with anti-windup */
    s_integrator += error * dt_sec;
    float i_contrib = s_ki * s_integrator;
    if (i_contrib > POS_KI_LIMIT_A) {
        s_integrator = POS_KI_LIMIT_A / s_ki;
        i_contrib = POS_KI_LIMIT_A;
    } else if (i_contrib < -POS_KI_LIMIT_A) {
        s_integrator = -POS_KI_LIMIT_A / s_ki;
        i_contrib = -POS_KI_LIMIT_A;
    }

    /* Derivative with low-pass filter (fc ~10Hz at 1kHz) */
    float d_raw = (dt_sec > 0.0f) ? (error - s_last_error) / dt_sec : 0.0f;
    s_deriv_filt += POS_DERIV_ALPHA * (d_raw - s_deriv_filt);
    s_last_error = error;

    /* PID output: negative sign so positive error -> corrective torque */
    float iq_cmd = -(s_kp * error + i_contrib + s_kd * s_deriv_filt);

    /* Clamp */
    if (iq_cmd > POS_IQ_LIMIT_A) iq_cmd = POS_IQ_LIMIT_A;
    if (iq_cmd < -POS_IQ_LIMIT_A) iq_cmd = -POS_IQ_LIMIT_A;

    return iq_cmd;
}
