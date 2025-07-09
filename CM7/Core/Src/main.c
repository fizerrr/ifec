/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pi_controller.h"
#include "pi_controller_current.h"
#include "no_load_controller.h"
#include "adc_config.h"
#include "ipc.h"
#include "power_on.h"
#include "ocp.h"




int start=0;
int test = 0;
int test_iqr = 0;
/* Zmienna do przechowywania wyniku ADC */
int target_duty=0;
/* Zmienna do pomiaru czasu */
float setpoint = 0;
float previous_setpoint = 0;
float kp_V_start=0.08f;
float ki_V_start=1.0f;
float kp_I_start=0.001f;
float ki_I_start=1.0f;
float V_reg_out;
float setpoint_I=2;
int current_case;
int plus=0;
int minus=0;
int zero=0;
float output_min_voltage=0;
float output_max_voltage=12;
float output_min_current=0;
float output_max_current=1;
float limit_low= -1200;
float limit_high= 1200;
float TS = 0.00005f;
float integral_set_V =20.5235348f;
float integral_set_I =0.695313931f;
float current_inductor_filter = 0.0f;
float old_current = 0.0f;
float voltage_out_filter=0.0f;
float old_voltage=0.0f;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Zmienna do przechowywania wyniku ADC */
uint16_t buck_fault,cec_fault, half_bridge_fault,ocp_state=0;


float current_inductor,current_out,voltage_out,voltage_in;

uint16_t duty = 0, duty_main = 750, dutych3=600, dutych2=900;
float set_voltage;
uint16_t state;

float output_max_current_ocp = 13;


__attribute__((section(".RAM_D2"))) uint16_t adc_buffer[2];
__attribute__((section(".RAM_D2"))) uint16_t adc2_buffer[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

float convert_adc_to_voltage(uint16_t adc_value);
float adc_to_voltage(uint16_t adc_value);
float adc_to_current_out(uint16_t adc_val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


PI_Controller pi_regulator;
PI_Controller_current pic_regulator;
NO_LOAD_CONTROLLER nol_regulator;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM13_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */



	 ADC1_Init_Custom(adc_buffer);
	 ADC2_Init_Custom(adc2_buffer);

	 Mesurments_Start();

	 PowerOnSequence_Start();

	 CEC_Start();

	 Fan_Start();

	 OCP_Init(output_max_current_ocp, 0.01f);

	 Buck_Start();

  Controll_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  	test++;

	  	voltage_out = convert_adc_to_voltage(adc_buffer[0]);
	  	voltage_in = adc_to_voltage(adc2_buffer[0]);
	  	current_out = adc_to_current_out(adc2_buffer[1]);


	    buck_fault = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_2);
	    cec_fault = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_9);
	    half_bridge_fault = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_4);



	    ocp_state = OCP_Check(current_out);

//	    setpoint = IPC_SHARED->nap_zadane;

	    state = IPC_SHARED->stan_przeksztaltnika;

	    IPC_SHARED->nap_wejsciowe = voltage_out;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSI);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(4);
  LL_RCC_PLL1_SetN(60);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {

  }
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_Init1msTick(480000000);

  LL_SetSystemCoreClock(480000000);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(32);
  LL_RCC_PLL2_SetN(100);
  LL_RCC_PLL2_SetP(4);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

}

/* USER CODE BEGIN 4 */

float convert_adc_to_voltage(uint16_t adc_value) {
    return 0.008313f * adc_value - 275.287f;
}

float adc_to_voltage(uint16_t adc_val) {
    return 0.0101158f * adc_val - 336.14f;
}


float adc_to_current_out(uint16_t adc_val) {
    return 0.0005119f * adc_val - 7.82f;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
