/*
 * power_on.c
 *
 *  Created on: Jul 8, 2025
 *      Author: birdd
 */


#include "power_on.h"


extern float offset_adc1_ch0, offset_adc1_ch1, offset_adc2_ch0, offset_adc2_ch1;
extern uint16_t adc_buffer[2];
extern uint16_t adc2_buffer[2];

static uint8_t current_deadtime = 255;
#define ADC_OFFSET_SAMPLES 1000


void PowerOnSequence_Start(void)
{
	LL_mDelay(3000);

	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);


	LL_TIM_OC_SetCompareCH1(TIM8, 750);
    LL_TIM_OC_SetCompareCH2(TIM8, 900);
    LL_TIM_OC_SetCompareCH3(TIM8, 600);



    LL_TIM_EnableAllOutputs(TIM8);
    LL_TIM_EnableCounter(TIM8);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);

    LL_TIM_EnableIT_UPDATE(TIM13);
    LL_TIM_EnableCounter(TIM13);

}

void PowerOnSequence_Update(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM13)) {

        LL_TIM_ClearFlag_UPDATE(TIM13);

        if (current_deadtime == 254) {
            LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
            LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
        }

        if (current_deadtime > 24) {
            current_deadtime--;
            TIM8->BDTR = (TIM8->BDTR & ~TIM_BDTR_DTG_Msk) | (current_deadtime & 0xFF);
        }

        if (current_deadtime == 24) {

            LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
            LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3N);
            LL_TIM_EnableAllOutputs(TIM8);
            LL_TIM_EnableCounter(TIM8);

            LL_TIM_DisableIT_UPDATE(TIM13);
            LL_TIM_DisableCounter(TIM13);
        }
    }
}

void Fan_Start(void)
{
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	  LL_TIM_EnableAllOutputs(TIM3);
	  LL_TIM_EnableCounter(TIM3);


	  LL_TIM_OC_SetCompareCH1(TIM3, 20000);
	  LL_TIM_OC_SetCompareCH2(TIM3, 20000);



}




void Buck_Start(void)
{
	  LL_TIM_OC_SetCompareCH1(TIM1, 0);
	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
	  LL_TIM_EnableAllOutputs(TIM1);
	  LL_TIM_EnableCounter(TIM1);
}



void CEC_Start(void)
{
	  LL_TIM_OC_SetCompareCH1(TIM2, 0);
	  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	  LL_TIM_EnableCounter(TIM2);

}


void Mesurments_Start(void)
{



	LL_TIM_OC_SetCompareCH3(TIM2, 2400);

    uint32_t sum_adc1_ch0 = 0;
    uint32_t sum_adc1_ch1 = 0;
    uint32_t sum_adc2_ch0 = 0;
    uint32_t sum_adc2_ch1 = 0;

    for (int i = 0; i < ADC_OFFSET_SAMPLES; i++)
    {
        LL_mDelay(1); // krótka przerwa, żeby ADC nadążył z DMA

        sum_adc1_ch0 += adc_buffer[0];
        sum_adc1_ch1 += adc_buffer[1];
        sum_adc2_ch0 += adc2_buffer[0];
        sum_adc2_ch1 += adc2_buffer[1];
    }

    offset_adc1_ch0 = sum_adc1_ch0 / (float)ADC_OFFSET_SAMPLES;
    offset_adc1_ch1 = sum_adc1_ch1 / (float)ADC_OFFSET_SAMPLES;
    offset_adc2_ch0 = sum_adc2_ch0 / (float)ADC_OFFSET_SAMPLES;
    offset_adc2_ch1 = sum_adc2_ch1 / (float)ADC_OFFSET_SAMPLES;


	LL_TIM_OC_SetCompareCH3(TIM2, 0);

}


