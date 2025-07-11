/*
 * ocp.h
 *
 *  Created on: Jul 8, 2025
 *      Author: birdd
 */

#ifndef __OCP_H__
#define __OCP_H__

void OCP_Init(float current_limit);
int OCP_Check(float current);
void OCP_Reset(float current, int state);


#endif

