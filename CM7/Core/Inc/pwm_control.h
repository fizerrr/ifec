/*
 * pwm_control.h
 *
 *  Created on: May 6, 2025
 *      Author: birdd
 */

#ifndef INC_PWM_CONTROL_H_
#define INC_PWM_CONTROL_H_

#include <stdint.h>

void PWM_SetDutyCycle(float duty);
void PWM_SetDeadTime_raw(uint8_t dtg);
void PWM_Init(void); // wywoływane raz na start

#endif /* INC_PWM_CONTROL_H_ */
