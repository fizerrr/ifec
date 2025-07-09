/*
 * ocp.h
 *
 *  Created on: Jul 8, 2025
 *      Author: birdd
 */

#ifndef __OCP_H__
#define __OCP_H__

void OCP_Init(float current_limit, float filter_coeff);
int OCP_Check(float current);
void OCP_Reset(void);


#endif

