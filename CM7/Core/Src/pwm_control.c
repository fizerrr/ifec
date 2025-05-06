/*
 * pwm_control.c
 *
 *  Created on: May 6, 2025
 *      Author: birdd
 */


#include "pwm_control.h"
#include "pwm_config.h"
#include "stm32h7xx_ll_tim.h"

void PWM_SetDutyCycle(float duty)
{
    uint32_t arr = PWM_GetARR();
    uint32_t ccr = (uint32_t)(duty * (arr + 1));
    LL_TIM_OC_SetCompareCH1(PWM_TIMER, ccr);
}


//void PWM_SetDeadTime_ns(uint32_t deadtime_ns)
//{
//    uint32_t clk = PWM_GetTimerClock();  // np. 240 MHz
//    uint32_t ticks = (deadtime_ns * clk) / 1000000000UL;
//    uint32_t dtg = 0;
//
//    if (ticks <= 127)
//    {
//        // Direct mode (resolution 1 tick)
//        dtg = ticks;
//    }
//    else if (ticks <= 254)
//    {
//        // DTG[7] = 1, DTG[6] = 0 → resolution 2 ticks
//        dtg = ((ticks - 64) / 2) | 0x80;
//    }
//    else if (ticks <= 504)
//    {
//        // DTG[7:6] = 11 → resolution 8 ticks
//        dtg = ((ticks - 32) / 8) | 0xC0;
//    }
//    else if (ticks <= 1008)
//    {
//        // DTG[7:5] = 111 → resolution 16 ticks
//        dtg = ((ticks - 32) / 16) | 0xE0;
//    }
//    else
//    {
//        dtg = 0xFF; // max
//    }
//
//    PWM_TIMER->BDTR = (PWM_TIMER->BDTR & ~TIM_BDTR_DTG_Msk) | dtg;
//}

void PWM_SetDeadTime_raw(uint8_t dtg)
{
    PWM_TIMER->BDTR = (PWM_TIMER->BDTR & ~TIM_BDTR_DTG_Msk) | dtg;
}

