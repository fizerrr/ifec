/*
 * pi_controller.h
 *
 *  Created on: Jun 4, 2025
 *      Author: birdd
 */

#ifndef INC_PI_CONTROLLER_H_
#define INC_PI_CONTROLLER_H_


#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Ts;      // Czas próbkowania [s]
    float integral;
    float out_min;
    float out_max;
} PI_Controller;

void PI_Init(PI_Controller *pi, float Kp, float Ki, float Ts, float out_min, float out_max);
float PI_Update(PI_Controller *pi, float setpoint, float measurement, float measurement_current);
void PI_Reset(PI_Controller *pi);

#endif /* INC_PI_CONTROLLER_H_ */
