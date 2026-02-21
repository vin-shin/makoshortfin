#include "FOC.h"
#include "config.h"
#include "stm32g4xx.h"
#include <math.h>

/* PI controller state */
static float s_id_ref = 0.0f;
static float s_iq_ref = 0.0f;
static float s_kp_d = FOC_KP_D;
static float s_ki_d = FOC_KI_D;
static float s_kp_q = FOC_KP_Q;
static float s_ki_q = FOC_KI_Q;
static float s_id_integrator = 0.0f;
static float s_iq_integrator = 0.0f;

/* Diagnostic exports */
volatile float g_foc_ia_raw = 0.0f;
volatile float g_foc_ib_raw = 0.0f;
volatile float g_foc_ic_raw = 0.0f;
volatile float g_foc_i_alpha = 0.0f;
volatile float g_foc_i_beta = 0.0f;
volatile float g_foc_elec_angle = 0.0f;
volatile float g_foc_sin_e = 0.0f;
volatile float g_foc_cos_e = 0.0f;
volatile float g_foc_iq_cmd = 0.0f;
volatile float g_foc_iq_meas = 0.0f;
volatile float g_foc_vq_output = 0.0f;
volatile uint32_t g_foc_diag_counter = 0;

void FOC_Init(void)
{
    /* Enable CORDIC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    FOC_Reset();
}

void FOC_Reset(void)
{
    s_id_integrator = 0.0f;
    s_iq_integrator = 0.0f;
}

void FOC_SetCurrentRefs(float id_ref, float iq_ref)
{
    s_id_ref = id_ref;
    s_iq_ref = iq_ref;
}

void FOC_SetGains(float kp_d, float ki_d, float kp_q, float ki_q)
{
    s_kp_d = kp_d;
    s_ki_d = ki_d;
    s_kp_q = kp_q;
    s_ki_q = ki_q;
}

/* CORDIC hardware sin/cos from 15-bit count [0, 32767].
 * [0, 32767] → [-16384, 16383] (subtract 16384) → Q31 (<< 17).
 * One subtract, one shift — no float, no wrapping, no clamp. */
void FOC_SinCos(uint16_t angle_counts, float *sin_out, float *cos_out)
{
    int32_t angle_q31 = ((int32_t)angle_counts - 16384) << 17;

    /* Configure CORDIC: cosine function, 5 iterations, 2 results */
    CORDIC->CSR = (0U << CORDIC_CSR_FUNC_Pos)
               | (5U << CORDIC_CSR_PRECISION_Pos)
               | CORDIC_CSR_NRES;

    CORDIC->WDATA = (uint32_t)angle_q31;

    /* Read cosine result */
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}
    int32_t cos_q31 = (int32_t)CORDIC->RDATA;

    /* Read sine result */
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}
    int32_t sin_q31 = (int32_t)CORDIC->RDATA;

    *cos_out = (float)cos_q31 * (1.0f / 2147483648.0f);
    *sin_out = (float)sin_q31 * (1.0f / 2147483648.0f);
}

/* Full FOC pipeline: Clarke -> Park -> PI -> Inv Park -> SVPWM */
void FOC_Run(const FOC_Sensors_t *sensors, FOC_Output_t *output)
{
    float ia = sensors->ia;
    float ib = sensors->ib;
    float ic = sensors->ic;
    float bus_v = sensors->bus_v;

#if FOC_ENABLE_DIAGNOSTICS
    /* Diagnostic capture */
    g_foc_ia_raw = ia;
    g_foc_ib_raw = ib;
    g_foc_ic_raw = ic;
    g_foc_iq_cmd = s_iq_ref;
#endif

    /* Safety: zero output if bus voltage invalid */
    if (bus_v <= 0.0f) {
        output->duty_u = 0.5f;
        output->duty_v = 0.5f;
        output->duty_w = 0.5f;
        output->id_meas = 0.0f;
        output->iq_meas = 0.0f;
        output->vd = 0.0f;
        output->vq = 0.0f;
        return;
    }

    /* Sin/cos of electrical angle via CORDIC */
    float sin_e, cos_e;
    uint16_t angle_counts = sensors->elec_counts;
#if INVERT_ENCODER_ANGLE
    angle_counts = (uint16_t)(32768U - angle_counts) & 0x7FFF;
#endif
    FOC_SinCos(angle_counts, &sin_e, &cos_e);

#if FOC_ENABLE_DIAGNOSTICS
    /* Diagnostic capture */
    g_foc_elec_angle = (float)angle_counts * (TWO_PI_F / 32768.0f);
    g_foc_sin_e = sin_e;
    g_foc_cos_e = cos_e;
#endif

    float i_sum = (ia + ib + ic) * (1.0f / 3.0f);
    ia -= i_sum;
    ib -= i_sum;
    ic -= i_sum;

    /* Clarke transform (power-invariant): 3-phase -> alpha-beta */
    const float clarke_k = 0.81649658f; /* sqrt(2/3) */
    float i_alpha = clarke_k * ia;
    float i_beta = clarke_k * (ia + 2.0f * ib) * INV_SQRT3_F;
    
#if FOC_ENABLE_DIAGNOSTICS
    /* Diagnostic capture */
    g_foc_i_alpha = i_alpha;
    g_foc_i_beta = i_beta;
#endif

    /* Park transform: alpha-beta -> d-q (rotating frame) */
    float id = i_alpha * cos_e + i_beta * sin_e;
    float iq = -i_alpha * sin_e + i_beta * cos_e;

    output->id_meas = id;
    output->iq_meas = iq;
    
#if FOC_ENABLE_DIAGNOSTICS
    /* Diagnostic capture */
    g_foc_iq_meas = iq;
#endif

    /* PI current controllers */
    float id_err = s_id_ref - id;
    float iq_err = s_iq_ref - iq;

    float v_limit = bus_v * SVPWM_V_LIMIT;

    /* D-axis PI with anti-windup */
    float vd = s_kp_d * id_err + s_id_integrator;

    /* Q-axis PI with anti-windup */
    float vq = s_kp_q * iq_err + s_iq_integrator;

    /* dq decoupling feedforward: cancel cross-coupling before voltage limit.
     * Vd -= ωe·L·iq  (cancels +ωe·L·iq disturbance in d-axis plant)
     * Vq += ωe·L·id  (cancels -ωe·L·id disturbance in q-axis plant) */
    float omega_e = sensors->omega_e;
    vd -= omega_e * MOTOR_PHASE_INDUCTANCE_H * iq;
    vq += omega_e * MOTOR_PHASE_INDUCTANCE_H * id;

    /* Circular voltage limiting (check if saturating) */
    float v_mag_sq = vd * vd + vq * vq;
    float v_lim_sq = v_limit * v_limit;
    int32_t saturated = (v_mag_sq > v_lim_sq) ? 1 : 0;
    
    if (saturated) {
        float scale = v_limit / sqrtf(v_mag_sq);
        vd *= scale;
        vq *= scale;

        /* Anti-windup: Do NOT integrate when saturated */
        /* Integrators are unchanged */
    } else {
        /* Only integrate when NOT saturated */
        s_id_integrator += s_ki_d * id_err * FOC_ISR_DT_S;
        s_iq_integrator += s_ki_q * iq_err * FOC_ISR_DT_S;
        
        /* Secondary clamp as safety net (shouldn't trigger often) */
        if (s_id_integrator > v_limit) s_id_integrator = v_limit;
        if (s_id_integrator < -v_limit) s_id_integrator = -v_limit;
        if (s_iq_integrator > v_limit) s_iq_integrator = v_limit;
        if (s_iq_integrator < -v_limit) s_iq_integrator = -v_limit;
    }

    output->vd = vd;
    output->vq = vq;
    
#if FOC_ENABLE_DIAGNOSTICS
    /* Diagnostic capture */
    g_foc_vq_output = vq;
    g_foc_diag_counter++;
#endif

    /* Inverse Park transform: d-q -> alpha-beta */
    float v_alpha = vd * cos_e - vq * sin_e;
    float v_beta = vd * sin_e + vq * cos_e;

    /* Inverse Clarke (power-invariant): alpha-beta -> 3-phase */
    const float inv_clarke_k = 1.2247449f; /* sqrt(3/2) */
    float vu = inv_clarke_k * v_alpha;
    float vv = inv_clarke_k * (-0.5f * v_alpha + 0.5f * SQRT3_F * v_beta);
    float vw = inv_clarke_k * (-0.5f * v_alpha - 0.5f * SQRT3_F * v_beta);

    /* Min-max injection (midpoint clamping) */
    float v_max = fmaxf(vu, fmaxf(vv, vw));
    float v_min = fminf(vu, fminf(vv, vw));
    float v_offset = 0.5f * (v_max + v_min);
    vu -= v_offset;
    vv -= v_offset;
    vw -= v_offset;

    /* Normalize to duty cycles [0, 1] */
    float inv_bus = 1.0f / bus_v;
    float du = 0.5f + vu * inv_bus;
    float dv = 0.5f + vv * inv_bus;
    float dw = 0.5f + vw * inv_bus;

    /* Clamp */
    if (du < 0.0f) du = 0.0f; else if (du > 1.0f) du = 1.0f;
    if (dv < 0.0f) dv = 0.0f; else if (dv > 1.0f) dv = 1.0f;
    if (dw < 0.0f) dw = 0.0f; else if (dw > 1.0f) dw = 1.0f;

    output->duty_u = du;
    output->duty_v = dv;
    output->duty_w = dw;
}
