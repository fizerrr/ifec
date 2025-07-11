/*
 * state_controller.c
 *
 *  Created on: Jul 11, 2025
 *      Author: birdd
 */


#include <controll.h>
#include "ocp.h"
#include "tim.h"     // jeśli używasz np. LL_TIM_EnableIT_UPDATE lub HAL_TIM_xxx
#include "ipc.h"
#include "controll.h"

static int controll_enabled = 0;

int temp=0;;

int State_Controller_Update(uint16_t state, uint16_t ocp_state)
{
    if (state == 1)
    {
        if (ocp_state == 0)
        {
            if (!controll_enabled)
            {
                Controll_Enable(); // Twoja funkcja włączająca timery i przerwania
                controll_enabled = 1;
            }
        }
        else
        {
            if (controll_enabled)
            {
                Controll_Disable(); // Twoja funkcja wyłączająca timery i przerwania
                controll_enabled = 0;
            }
        }
    }
    else
    {
        if (controll_enabled)
        {
            Controll_Disable();
            controll_enabled = 0;
        }
    }

    return controll_enabled;
}


void Controll_Enable(void)
{
	LL_TIM_OC_SetCompareCH1(TIM2, 0);

	LL_TIM_EnableCounter(TIM4);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
    LL_TIM_EnableIT_UPDATE(TIM4);

    temp++;
}
void Controll_Disable(void)
{

	LL_TIM_DisableIT_UPDATE(TIM4);
	LL_TIM_DisableCounter(TIM4);
	LL_TIM_OC_SetCompareCH1(TIM1, 0);
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM8, LL_TIM_CHANNEL_CH1N);

    LL_TIM_OC_SetCompareCH1(TIM2, 600);

}
