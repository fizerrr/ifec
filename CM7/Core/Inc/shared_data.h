/*
 * shared_data.h
 *
 *  Created on: Jun 28, 2025
 *      Author: birdd
 */

#ifndef INC_SHARED_DATA_H_
#define INC_SHARED_DATA_H_


#include <stdint.h>

typedef struct __attribute__((packed)) {
    volatile uint8_t output_state;
    volatile float voltage_set;
    volatile float voltage_measured;
    volatile uint32_t update_counter;
} SharedData_t;

// Wskaż dokładny adres w RAM_D2 (domyślnie: 0x30000000)
#define SHARED_DATA_BASE ((SharedData_t*)0x30000000U)

#define shared_data (*(SHARED_DATA_BASE))

#endif
