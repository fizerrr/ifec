/*
 * power_on.h
 *
 *  Created on: Jul 8, 2025
 *      Author: birdd
 */

#ifndef INC_POWER_ON_H_
#define INC_POWER_ON_H_

#include "main.h"

void PowerOnSequence_Start(void);
void PowerOnSequence_Update(void);
void Fan_Start(void);
void Buck_Start(void);
void CEC_Start(void);
void Mesurments_Start(void);

#endif /* INC_POWER_ON_H_ */
