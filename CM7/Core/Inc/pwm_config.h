/*
 * pwm_config.h
 *
 *  Created on: May 6, 2025
 *      Author: birdd
 */

#ifndef INC_PWM_CONFIG_H_
#define INC_PWM_CONFIG_H_

#include <stdint.h>
#include "stm32h7xx.h"

uint32_t PWM_GetTimerClock(void);
uint32_t PWM_GetARR(void);

extern TIM_TypeDef *PWM_TIMER;

#endif /* INC_PWM_CONFIG_H_ */
