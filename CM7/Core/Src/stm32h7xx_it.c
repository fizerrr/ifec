/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pi_controller.h"
#include "pi_controller_current.h"
#include "no_load_controller.h"
#include "power_on.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
float KP_V_dynamic=5;
float KP_I_dynamic=0.1;
float KP_V_dynamic2=0.1f;
float KP_I_dynamic2=0.00001f;
extern float ki_V_start;
extern int start;
extern int current_case;
int zmiana_stanu;
extern int test_iqr;
extern int plus;
extern int minus;
extern int zero;
extern PI_Controller pi_regulator;
extern PI_Controller_current pic_regulator;
extern NO_LOAD_CONTROLLER nol_regulator;
extern int target_duty;
extern float setpoint;
extern float previous_setpoint;
//extern float voltage_out;
//extern float voltage_in;
extern float V_reg_out;
//extern float current_out;
extern float kp_V_start;
extern float kp_I_start;
extern float current_inductor_filter;
extern float old_current;
extern uint16_t buck_fault;
int entered_case1_once = 0; // Flaga stanu
int xd=0;
float Kp_pi_nominal;
float Kp_pic_nominal;
int force_case1_counter = 0;
volatile uint8_t blokada_case_change = 0;
int kp_boost_counter = 0;
extern float current_inductor;
extern float setpoint_I;
extern float integral_set_V;
extern float integral_set_I;
extern float voltage_out_filter;
extern float old_voltage;
float current_triger;

int target_duty_sum= 0;



extern float  current_out_adc,voltage_in_adc, voltage_out_adc;
extern float voltage_out_adc_out, current_inductor_adc_out;


float setpoint_underload_correction_value[13] = {
    4.63000011f,
    4.51999998f,
    4.59999990f,
    4.82000017f,
    4.82000017f,
    4.82000017f,
    4.69999981f,
    5.0f,
    5.0f,
    4.69999981f,
    4.69999981f,
    5.69999981f,
    5.0f
};


//extern float voltage_out_adc_old, current_inductor_adc_old, voltage_in_adc_old, current_out_adc_old;


//float coff = 0.001;
//float coff1 =0.1;


int xddd=420;
#define MANUAL_DUTY_CYCLES 30
int manual_duty_counter = 0;
int manual_duty_array[MANUAL_DUTY_CYCLES] = {
    400, 350, 320, 280, 100, 50,
    140, 150, 150, 150,
    145, 145, 140, 140, 140, 140, 140, 140, 140, 140,
    140, 140, 140, 140, 140, 140, 140, 140, 140, 140
};

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void filter(float *out, float *old, float coff, float in);
int return_safe_correction_index(float index);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
	static int previous_case = -1;

    if (LL_TIM_IsActiveFlag_UPDATE(TIM4)) {
        LL_TIM_ClearFlag_UPDATE(TIM4);
       // LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
      //  LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_0);



        //filter(&voltage_in_adc_out,        &voltage_in_adc_old,        coff, voltage_in_adc);
      //  filter(&current_out_adc_out,       &current_out_adc_old,       0.001, current_out_adc);




        if (previous_setpoint != setpoint) {
            previous_setpoint = setpoint;
            blokada_case_change = 400; // blokuj zmianę current_case przez 20 cykli
        }

        if (blokada_case_change > 0) {
            blokada_case_change--;
        } else if (force_case1_counter > 0) {
            force_case1_counter--;
            current_case = 1;
        } else {
            current_case = (current_out_adc>= 1.5f && target_duty >= 0) ? 1 : 0;
        }

       // current_case =1;

        switch (current_case) {
            case 0:

            	 if (previous_case == 1) {
            	                    nol_regulator.previous_setpoint = 151.0f;

            	                }
                // Reset flagi po wyjściu z case 1
                entered_case1_once = 0;


                // Brak obciążenia – np. pomiń regulator prądu
                LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
                target_duty = (int)NOL_Update(&nol_regulator, setpoint, voltage_out_adc, voltage_in_adc);

                switch ((target_duty > 0) ? 1 : (target_duty < 0) ? -1 : 0) {
                    case 0:
                        LL_TIM_OC_SetCompareCH1(TIM1, 0);
                        LL_TIM_OC_SetCompareCH1(TIM2, 0);
                        //zero++;
                        break;

                    case 1:
                        LL_TIM_OC_SetCompareCH1(TIM1, target_duty);
                        LL_TIM_OC_SetCompareCH1(TIM2, 0);
                        plus++;
                        break;

                    case -1:
                        LL_TIM_OC_SetCompareCH1(TIM1, 0);
                        LL_TIM_OC_SetCompareCH1(TIM2, -target_duty);
                        minus++;
                        break;
                }

                break;

                case 1: {
                    static int case1_divider = 0;

                    // Jednorazowe wejście
                    if (!entered_case1_once)
                    {
                        entered_case1_once = 1;
                       // force_case1_counter = 1000;
                        force_case1_counter = 100000;
                      //  force_case1_counter = 100000000000;
                        xd++;
                        current_triger=current_out_adc;
                        LL_TIM_OC_SetCompareCH1(TIM2, 0);
                        LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

                        PI_SetIntegralLimit(&pi_regulator, 10/ki_V_start);
                        PIC_SetIntegralLimit(&pic_regulator, setpoint/180);
                        V_reg_out=10;

                        target_duty_sum = 0;
                        kp_boost_counter = 0;

                       // case1_divider = 0; // Wyzeruj licznik przy wejściu!
                    }

                    if (kp_boost_counter < 5) {
                        pi_regulator.Kp = KP_V_dynamic;
                        pic_regulator.Kp = KP_I_dynamic;
                        //kp_boost_counter++;
                    } else if (kp_boost_counter <10) {
                    	pi_regulator.Kp = KP_V_dynamic2;
                    	pic_regulator.Kp = KP_I_dynamic2;
                    	//kp_boost_counter++;
                    } else {
                    	pi_regulator.Kp = kp_V_start;
                    	pic_regulator.Kp = kp_I_start;
                    }


                    // Co 10 cykli (czyli 20 kHz), lub od razu po wejściu

                  //  pi_regulator.Kp = kp_V_start;
                  // 	pic_regulator.Kp = kp_I_start;



                     kp_boost_counter++;

                    V_reg_out = PI_Update(&pi_regulator, setpoint + setpoint_underload_correction_value[ return_safe_correction_index(( setpoint / 10 ) - 2) ], voltage_out_adc_out, 0);

                    target_duty = (int)PIC_Update(&pic_regulator, V_reg_out, current_inductor_adc_out);

                   if(kp_boost_counter > 200000 && kp_boost_counter < 200000+200) target_duty_sum += target_duty;

                    if( kp_boost_counter >  200000+200) target_duty = target_duty_sum/200;


                   //	target_duty=xddd;
           LL_TIM_OC_SetCompareCH1(TIM1, target_duty);
           LL_TIM_OC_SetCompareCH1(TIM2, 0);

        }

                break;
        }

        previous_case = current_case;
    }


   // LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
  //  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_0);

}


/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */


/* USER CODE BEGIN 1 */

//
//void filter(float *out,float *old, float coff, float in)
//{
//    *out= coff*in +(1-coff)*(*old);
//    *old=*out;
//
//}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
	PowerOnSequence_Update();
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/* USER CODE BEGIN 1 */


/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */

/* USER CODE BEGIN 1 */


int return_safe_correction_index(float index)
{
    if (index >= 0 && index <= 13 && index == (int)index)
    {
        return (int)index;
    }

    return 0;
}


/* USER CODE END 1 */
