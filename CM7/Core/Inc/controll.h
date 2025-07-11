/*
 * state_controller.h
 *
 *  Created on: Jul 11, 2025
 *      Author: birdd
 */

#ifndef INC_CONTROLL_H_
#define INC_CONTROLL_H_


#include <stdint.h>

int State_Controller_Update(uint16_t state, uint16_t ocp_state);


void Controll_Enable(void);
void Controll_Disable(void);


#endif /* INC_CONTROLL_H_ */
