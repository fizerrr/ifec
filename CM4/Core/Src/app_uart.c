/*
 * app_uart.c
 *
 *  Created on: Jun 25, 2025
 *      Author: birdd
 */


#include "app_uart.h"
#include "ring_buffer.h"
#include "command_parser.h"
#include "usart.h"
#include "main.h"

#include <string.h>

#define LINE_BUFFER_SIZE 32

static char line_buffer[LINE_BUFFER_SIZE];
static uint8_t line_index = 0;

// Zmienna dostępna z main.c:
extern char command_buffer[LINE_BUFFER_SIZE];
extern volatile bool command_ready;


void App_UART_Init(void) {
    LL_USART_EnableIT_RXNE(UART4);
}

void App_UART_IRQHandler(void) {
    if (LL_USART_IsActiveFlag_RXNE(UART4)) {
        uint8_t byte = LL_USART_ReceiveData8(UART4);


        if (byte == '\r' || byte == '\n') {
            if (line_index > 0 && !command_ready) {
                line_buffer[line_index] = '\0';
                strncpy(command_buffer, line_buffer, LINE_BUFFER_SIZE);
                command_ready = true;
            }
            line_index = 0; // reset bufora niezależnie
        } else if (line_index < LINE_BUFFER_SIZE - 1) {
            line_buffer[line_index++] = byte;
        } else {
            line_index = 0; // overflow: resetujemy
        }
    }
}
