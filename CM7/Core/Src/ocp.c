#include "ocp.h"
#include "tim.h"  // jeśli używasz LL_TIM_CC_DisableChannel()

static float filtered_current = 0.0f;
static float old_current = 0.0f;
static float ocp_limit = 1.0f;
static float ocp_filter_coeff = 0.05f;
static int ocp_triggered = 0;

static void filter(float *out, float *old, float coff, float in)
{
    *out = coff * in + (1.0f - coff) * (*old);
    *old = *out;
}

void OCP_Init(float current_limit, float filter_coeff)
{
    ocp_limit = current_limit;
    ocp_filter_coeff = filter_coeff;
    filtered_current = 0.0f;
    old_current = 0.0f;
    ocp_triggered = 0;
}

int OCP_Check(float current)
{
    filter(&filtered_current, &old_current, ocp_filter_coeff, current);

    if (!ocp_triggered && filtered_current > ocp_limit)
    {
        // Wyłącz PWM lub zareaguj

        LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH1);
        LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
        ocp_triggered = 1;
    }

    return ocp_triggered;
}

void OCP_Reset(void)
{
    filtered_current = 0.0f;
    old_current = 0.0f;
    ocp_triggered = 0;
}

