/*
 * pwm_config.c
 *
 *  Created on: May 6, 2025
 *      Author: birdd
 */

#include "pwm_config.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_tim.h"

TIM_TypeDef *PWM_TIMER = TIM1;
static uint32_t timer_clock_hz = 0;
static uint32_t pwm_arr = 0;

void PWM_Init(void)
{
    if (timer_clock_hz == 0)
    {
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        if (LL_RCC_GetAPB2Prescaler() != LL_RCC_APB2_DIV_1)
            timer_clock_hz = pclk * 2;
        else
            timer_clock_hz = pclk;

        // Ustal ARR dla danej częstotliwości np. 100 kHz
        uint32_t target_freq = 200000; // Możesz to też przekazać jako parametr
        pwm_arr = (timer_clock_hz / target_freq) - 1;
        LL_TIM_SetAutoReload(PWM_TIMER, pwm_arr);
        LL_TIM_OC_SetCompareCH1(PWM_TIMER, pwm_arr / 2); // domyślne 50%
        LL_TIM_GenerateEvent_UPDATE(PWM_TIMER);
    }
}

uint32_t PWM_GetTimerClock(void)
{
    return timer_clock_hz;
}

uint32_t PWM_GetARR(void)
{
    return pwm_arr;
}


