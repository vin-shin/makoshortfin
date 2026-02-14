#include "FOC.h"
#include <math.h>

#define FOC_PI 3.14159265358979323846f
#define FOC_INV_SQRT3 0.57735026918962576451f
#define FOC_SQRT3 1.73205080756887729353f
#define FOC_SVPWM_VLIMIT 0.57735026918962576451f
#define FOC_CORDIC_PRECISION 5U
#define FOC_CORDIC_FUNC_COS 0x0U

static uint16_t s_pole_pairs = 1;
static float s_ia_offset = 0.0f;
static float s_ib_offset = 0.0f;
static float s_ic_offset = 0.0f;
static FOC_SensorData s_data;
static float s_id_ref = 0.0f;
static float s_iq_ref = 0.0f;
static float s_kp_d = 0.2f;
static float s_ki_d = 50.0f;
static float s_kp_q = 0.2f;
static float s_ki_q = 50.0f;
static float s_dt = 0.00005f;
static float s_id_integrator = 0.0f;
static float s_iq_integrator = 0.0f;

static float FOC_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static int32_t FOC_FloatToQ31(float value)
{
  if (value >= 0.999999999f)
  {
    value = 0.999999999f;
  }
  if (value <= -1.0f)
  {
    value = -1.0f;
  }
  return (int32_t)(value * 2147483648.0f);
}

static float FOC_Q31ToFloat(int32_t value)
{
  return ((float)value) / 2147483648.0f;
}

static float FOC_WrapAngle(float angle)
{
  float wrapped = fmodf(angle + FOC_PI, 2.0f * FOC_PI);
  if (wrapped < 0.0f)
  {
    wrapped += 2.0f * FOC_PI;
  }
  return wrapped - FOC_PI;
}

static void FOC_CordicSinCos(float angle, float *sin_out, float *cos_out)
{
  float angle_norm = FOC_WrapAngle(angle) / FOC_PI;
  int32_t angle_q31 = FOC_FloatToQ31(angle_norm);

  uint32_t csr = (FOC_CORDIC_FUNC_COS << CORDIC_CSR_FUNC_Pos)
                 | (FOC_CORDIC_PRECISION << CORDIC_CSR_PRECISION_Pos)
                 | CORDIC_CSR_NRES;

  CORDIC->CSR = csr;
  CORDIC->WDATA = (uint32_t)angle_q31;

  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U)
  {
  }
  int32_t cos_q31 = (int32_t)CORDIC->RDATA;

  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U)
  {
  }
  int32_t sin_q31 = (int32_t)CORDIC->RDATA;

  if (cos_out != NULL)
  {
    *cos_out = FOC_Q31ToFloat(cos_q31);
  }
  if (sin_out != NULL)
  {
    *sin_out = FOC_Q31ToFloat(sin_q31);
  }
}

void FOC_SinCos(float angle, float *sin_out, float *cos_out)
{
  FOC_CordicSinCos(angle, sin_out, cos_out);
}

void FOC_Init(void)
{
  __HAL_RCC_CORDIC_CLK_ENABLE();
  FOC_Reset();
}

void FOC_Reset(void)
{
  s_pole_pairs = 1;
  s_ia_offset = 0.0f;
  s_ib_offset = 0.0f;
  s_ic_offset = 0.0f;
  s_data.ia = 0.0f;
  s_data.ib = 0.0f;
  s_data.ic = 0.0f;
  s_data.bus_v = 0.0f;
  s_data.electrical_angle = 0.0f;
  s_id_ref = 0.0f;
  s_iq_ref = 0.0f;
  s_id_integrator = 0.0f;
  s_iq_integrator = 0.0f;
}

void FOC_ResetIntegrators(void)
{
  s_id_integrator = 0.0f;
  s_iq_integrator = 0.0f;
}

void FOC_SetPolePairs(uint16_t pole_pairs)
{
  if (pole_pairs == 0)
  {
    pole_pairs = 1;
  }
  s_pole_pairs = pole_pairs;
}

void FOC_SetCurrentOffsets(float ia_offset, float ib_offset, float ic_offset)
{
  s_ia_offset = ia_offset;
  s_ib_offset = ib_offset;
  s_ic_offset = ic_offset;
}

void FOC_SetCurrentRefs(float id_ref, float iq_ref)
{
  s_id_ref = id_ref;
  s_iq_ref = iq_ref;
}

void FOC_SetCurrentGains(float kp_d, float ki_d, float kp_q, float ki_q)
{
  s_kp_d = kp_d;
  s_ki_d = ki_d;
  s_kp_q = kp_q;
  s_ki_q = ki_q;
}

void FOC_SetControlPeriod(float dt_seconds)
{
  if (dt_seconds > 0.0f)
  {
    s_dt = dt_seconds;
  }
}

void FOC_UpdateSensors(const FOC_SensorData *data)
{
  if (data == NULL)
  {
    return;
  }
  s_data = *data;
  s_data.ia -= s_ia_offset;
  s_data.ib -= s_ib_offset;
  s_data.ic -= s_ic_offset;
}

void FOC_Run(FOC_Output *out)
{
  if (out == NULL)
  {
    return;
  }

  if (!isfinite(s_data.ia) || !isfinite(s_data.ib) || !isfinite(s_data.ic)
      || !isfinite(s_data.bus_v) || !isfinite(s_data.electrical_angle))
  {
    out->id_ref = s_id_ref;
    out->iq_ref = s_iq_ref;
    out->vd = 0.0f;
    out->vq = 0.0f;
    out->duty_u = 0.5f;
    out->duty_v = 0.5f;
    out->duty_w = 0.5f;
    return;
  }

  float bus_v = s_data.bus_v;
  if (bus_v <= 0.0f)
  {
    out->id_ref = s_id_ref;
    out->iq_ref = s_iq_ref;
    out->vd = 0.0f;
    out->vq = 0.0f;
    out->duty_u = 0.5f;
    out->duty_v = 0.5f;
    out->duty_w = 0.5f;
    return;
  }

  float elec_angle = FOC_WrapAngle(s_data.electrical_angle);

  float sin_t = 0.0f;
  float cos_t = 1.0f;
  FOC_CordicSinCos(elec_angle, &sin_t, &cos_t);

  float ia = s_data.ia;
  float ib = s_data.ib;

  float i_alpha = ia;
  float i_beta = (ia + 2.0f * ib) * FOC_INV_SQRT3;

  float i_d = i_alpha * cos_t + i_beta * sin_t;
  float i_q = -i_alpha * sin_t + i_beta * cos_t;

  float id_error = s_id_ref - i_d;
  float iq_error = s_iq_ref - i_q;

  float v_integrator_limit = bus_v * FOC_SVPWM_VLIMIT;

  s_id_integrator += s_ki_d * id_error * s_dt;
  s_id_integrator = FOC_Clamp(s_id_integrator, -v_integrator_limit, v_integrator_limit);

  s_iq_integrator += s_ki_q * iq_error * s_dt;
  s_iq_integrator = FOC_Clamp(s_iq_integrator, -v_integrator_limit, v_integrator_limit);

  float v_d = s_kp_d * id_error + s_id_integrator;
  float v_q = s_kp_q * iq_error + s_iq_integrator;

  float v_mag_sq = v_d * v_d + v_q * v_q;
  float v_limit_sq = v_integrator_limit * v_integrator_limit;
  if (v_mag_sq > v_limit_sq && v_mag_sq > 0.0f)
  {
    float v_mag = sqrtf(v_mag_sq);
    float scale = v_integrator_limit / v_mag;
    v_d *= scale;
    v_q *= scale;
  }

  float v_alpha = v_d * cos_t - v_q * sin_t;
  float v_beta = v_d * sin_t + v_q * cos_t;

  float v_u = v_alpha;
  float v_v = -0.5f * v_alpha + 0.5f * FOC_SQRT3 * v_beta;
  float v_w = -0.5f * v_alpha - 0.5f * FOC_SQRT3 * v_beta;

  float v_max = fmaxf(v_u, fmaxf(v_v, v_w));
  float v_min = fminf(v_u, fminf(v_v, v_w));
  float v_offset = 0.5f * (v_max + v_min);

  v_u -= v_offset;
  v_v -= v_offset;
  v_w -= v_offset;

  float inv_bus = 1.0f / bus_v;

  out->id_ref = s_id_ref;
  out->iq_ref = s_iq_ref;
  out->vd = v_d;
  out->vq = v_q;
  out->duty_u = FOC_Clamp(0.5f + v_u * inv_bus, 0.0f, 1.0f);
  out->duty_v = FOC_Clamp(0.5f + v_v * inv_bus, 0.0f, 1.0f);
  out->duty_w = FOC_Clamp(0.5f + v_w * inv_bus, 0.0f, 1.0f);
}
