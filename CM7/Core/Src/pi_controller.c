/*
 * pi_controller.c
 *
 *  Created on: Jun 4, 2025
 *      Author: birdd
 */


#include "pi_controller.h"

void PI_Init(PI_Controller *pi, float Kp, float Ki, float Ts, float out_min, float out_max) {
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->Ts = Ts;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

float PI_Update(PI_Controller *pi, float setpoint, float measurement_voltage, float measurement_current) {
    float error = setpoint - measurement_voltage;

    float vin = 180;

    float dff = ( setpoint * pi->out_max ) / vin;

    // Człon całkujący
    pi->integral += error * pi->Ts;

    // Anti-windup
    if (pi->integral * pi->Ki > pi->out_max)
        pi->integral = pi->out_max / pi->Ki;
    else if (pi->integral * pi->Ki < pi->out_min)
        pi->integral = pi->out_min / pi->Ki;

    // Wyjście regulatora
    float output = pi->Kp * error + pi->Ki * pi->integral;

    // Ograniczenie
    if (output > pi->out_max) output = pi->out_max;
    if (output < pi->out_min) output = pi->out_min;

//    if(measurement_current > 10 )
//    {
//    	return 120;
//    }

    return output;
}

void PI_Reset(PI_Controller *pi) {
    pi->integral = 0.0f;
}
