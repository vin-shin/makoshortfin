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

/* CORDIC hardware sin/cos - Q31 format, 5 iterations */
void FOC_SinCos(float angle_rad, float *sin_out, float *cos_out)
{
    /* Wrap angle to [-pi, pi] then normalize to [-1, 1] for Q31 */
    float wrapped = fmodf(angle_rad + PI_F, TWO_PI_F);
    if (wrapped < 0.0f) wrapped += TWO_PI_F;
    wrapped -= PI_F;

    float norm = wrapped * (1.0f / PI_F);

    /* Clamp to valid Q31 range */
    if (norm > 0.999999f) norm = 0.999999f;
    if (norm < -1.0f) norm = -1.0f;

    int32_t angle_q31 = (int32_t)(norm * 2147483648.0f);

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
    FOC_SinCos(sensors->elec_angle, &sin_e, &cos_e);

    float i_sum = (ia + ib + ic) * (1.0f / 3.0f);
    ia -= i_sum;
    ib -= i_sum;
    ic -= i_sum;

    /* Clarke transform: 3-phase -> alpha-beta */
    float i_alpha = ia;
    float i_beta = (ia + 2.0f * ib) * INV_SQRT3_F;

    /* Park transform: alpha-beta -> d-q (rotating frame) */
    float id = i_alpha * cos_e + i_beta * sin_e;
    float iq = -i_alpha * sin_e + i_beta * cos_e;

    output->id_meas = id;
    output->iq_meas = iq;

    /* PI current controllers */
    float id_err = s_id_ref - id;
    float iq_err = s_iq_ref - iq;

    float v_limit = bus_v * SVPWM_V_LIMIT;

    /* D-axis PI */
    s_id_integrator += s_ki_d * id_err * FOC_ISR_DT_S;
    if (s_id_integrator > v_limit) s_id_integrator = v_limit;
    if (s_id_integrator < -v_limit) s_id_integrator = -v_limit;
    float vd = s_kp_d * id_err + s_id_integrator;

    /* Q-axis PI */
    s_iq_integrator += s_ki_q * iq_err * FOC_ISR_DT_S;
    if (s_iq_integrator > v_limit) s_iq_integrator = v_limit;
    if (s_iq_integrator < -v_limit) s_iq_integrator = -v_limit;
    float vq = s_kp_q * iq_err + s_iq_integrator;

    /* Circular voltage limiting */
    float v_mag_sq = vd * vd + vq * vq;
    float v_lim_sq = v_limit * v_limit;
    if (v_mag_sq > v_lim_sq) {
        float scale = v_limit / sqrtf(v_mag_sq);
        vd *= scale;
        vq *= scale;
    }

    output->vd = vd;
    output->vq = vq;

    /* Inverse Park transform: d-q -> alpha-beta */
    float v_alpha = vd * cos_e - vq * sin_e;
    float v_beta = vd * sin_e + vq * cos_e;

    /* Inverse Clarke + SVPWM */
    float vu = v_alpha;
    float vv = -0.5f * v_alpha + 0.5f * SQRT3_F * v_beta;
    float vw = -0.5f * v_alpha - 0.5f * SQRT3_F * v_beta;

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
