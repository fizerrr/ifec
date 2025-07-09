#ifndef IPC_H
#define IPC_H

#include <stdint.h>

typedef struct {
    float nap_zadane;
    float nap_wejsciowe;
    uint8_t stan_przeksztaltnika;
    volatile uint8_t data_ready; // 1 - dane gotowe, 0 - dane odebrane
} IPC_Data;

// adres w SRAM4 (dostępna dla obu rdzeni)
#define IPC_SHARED ((IPC_Data *)0x38000000)

#endif
