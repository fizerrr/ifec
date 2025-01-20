/*
 * uart.h
 *
 *  Created on: Jan 19, 2025
 *      Author: birdd
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include "stm32h7xx_hal.h"

#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 100
#endif


extern UART_HandleTypeDef huart4;
extern char rxBuffer[RX_BUFFER_SIZE];

void UART_Init(UART_HandleTypeDef *huart, uint32_t baudRate);
void UART_Transmit(UART_HandleTypeDef *huart, char *string);
void UART_Receive(UART_HandleTypeDef *huart, char *buffer, uint16_t size);
void UART_Receive_IT(UART_HandleTypeDef *huart, char *buffer, uint16_t size);



#endif /* INC_UART_H_ */
