/*
 * adc_config.h
 *
 *  Created on: Jun 28, 2025
 *      Author: birdd
 */

#ifndef INC_ADC_CONFIG_H_
#define INC_ADC_CONFIG_H_

#include "main.h"

void ADC1_Init_Custom(uint16_t *buffer);
void ADC2_Init_Custom(uint16_t *buffer);
void ADC3_Init_Custom(uint16_t *buffer);
float adc_to_voltage_out(float adc_value);
float adc_to_current_inductor(float adc_value);
float adc_to_current_output(float adc_value);

#endif /* INC_ADC_CONFIG_H_ */
