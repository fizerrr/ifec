#include "ocp.h"
#include "tim.h"


static float ocp_limit = 13.0f;
static int ocp_triggered = 0;
static int states[2];


void OCP_Init(float current_limit)
{
    ocp_limit = current_limit;
    ocp_triggered = 0;
}

int OCP_Check(float current)
{


    if (!ocp_triggered && current > ocp_limit)
    {
        ocp_triggered = 1;

        LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_0);
    }

    return ocp_triggered;

}

void OCP_Reset(float current, int state)
{

    states[1] = states[0];

    states[0] = state;


    if (ocp_triggered && current < ocp_limit && !states[0] && states[1])
	{

    ocp_triggered = 0;

    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_0);

	}



}

