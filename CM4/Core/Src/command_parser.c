/*
 * command_parser.c
 *
 *  Created on: Jun 25, 2025
 *      Author: birdd
 */

#include "command_parser.h"
#include "usart.h"           // provides UART4 instance and LL_USART functions
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "ipc.h"  // dostęp do struktury dzielonej pamięci


// Internal state
extern int output_enable;
extern float vset;
extern float voltage_measured;

/**
 * @brief  Low-level send of a single character over UART4
 */
static void uart_send_char(char c) {
    // Wait until TXE (Transmit Data Register Empty)
    while (!LL_USART_IsActiveFlag_TXE(UART4));
    LL_USART_TransmitData8(UART4, (uint8_t)c);
}

/**
 * @brief  Send a NUL-terminated string over UART4
 */
static void uart_send_str(const char *s) {
    while (*s) {
        uart_send_char(*s++);
    }
}

/**
 * @brief  Respond with current output state: '0' or '1' followed by '\n'
 */
static void response_output_enable(void) {
	char buff[32] = {0};


	snprintf(buff, sizeof(buff), "%d\n", output_enable);

    uart_send_str(buff);
}

/**
 * @brief  Respond with set voltage (vset) formatted as X.YY\n
 */
static void response_vset(void) {
    char buf[32];
    // snprintf rounds correctly (adds +0.005)
    int intPart = (int)vset;
    int fracPart = (int)((vset - intPart + 0.005) * 100);

    snprintf(buf, sizeof(buf), "%d,%02d\n", intPart, abs(fracPart));
    uart_send_str(buf);
}

/**
 * @brief  Respond with measured voltage via external ADC function
 */
static void response_vmeas(void) {
    /*extern float get_measured_voltage(void);*/
    float vmeas = voltage_measured;
    char buf[32];
    int intPart = (int)vmeas;
    int fracPart = (int)((vmeas - intPart + 0.005) * 100);

    snprintf(buf, sizeof(buf), "%d,%02d\n", intPart, abs(fracPart));
    uart_send_str(buf);
}

/**
 * @brief  Parse and handle a received command line
 * @param  line: NUL-terminated string without CR/LF
 */
void command_parser_process_line(const char *line) {
    if (line == NULL || *line == '\0')
        return;

    // Skip leading whitespace
    while (isspace((unsigned char)*line))
        line++;

    // OUTPUT? -> query output state
    if (strncasecmp(line, "OUTPUT?", 7) == 0) {
        response_output_enable();
        return;
    }

    // OUTPUT <0|1|ON|OFF>
    if (strncasecmp(line, "OUTPUT ", 7) == 0) {
        const char *arg = line + 7;
        if (strncasecmp(arg, "ON", 2) == 0) {
            output_enable = 1;
        } else if (strncasecmp(arg, "OFF", 3) == 0) {
            output_enable = 0;
        } else {
            int val = atoi(arg);
            if (val == 0 || val == 1)
                output_enable = val;
        }
        return;
    }

    // VOLT? -> query set voltage
    if (strncasecmp(line, "VOLT?", 5) == 0) {
        response_vset();
        return;
    }

    // VOLT <float>
    if (strncasecmp(line, "VOLT ", 5) == 0) {
        float tmp = strtof(line + 5, NULL);
        vset = tmp;
        return;
    }

    // FETCH:VOLT? or FETC:VOLT? -> query measured voltage
    if (strncasecmp(line, "FETCH:VOLT?", 11) == 0 ||
        strncasecmp(line, "FETC:VOLT?", 10) == 0) {
        response_vmeas();
        return;
    }

    // Unknown command: ignore or implement error response here
}
