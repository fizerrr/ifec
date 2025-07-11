//#include "adc_config.h"
//
//void ADC1_Init_Custom(uint16_t *buffer) {
//    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)&ADC1->DR);
//    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)buffer);
//    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, 1);
//    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
//
//    ADC1->CFGR &= ~ADC_CFGR_DMNGT;
//    ADC1->CFGR |= (ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1);  // DMA circular
//
//
//    LL_ADC_Enable(ADC1);
//    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
//    LL_ADC_REG_StartConversion(ADC1);
//}
//
//void ADC2_Init_Custom(uint16_t *buffer) {
//    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&ADC2->DR);
//    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)buffer);
//    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, 2); // 2 kanały!
//    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
//
//    ADC2->CFGR &= ~ADC_CFGR_DMNGT;
//    ADC2->CFGR |= (ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1);  // DMA circular
//
//
//    LL_ADC_Enable(ADC2);
//    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2));
//    LL_ADC_REG_StartConversion(ADC2);
//}

#include "adc_config.h"
#include <stdint.h>

void ADC1_Init_Custom(uint16_t *buffer) {
    // Kalibracja w trybie różnicowym
    LL_ADC_StartCalibration(ADC1, LL_ADC_CALIB_OFFSET, LL_ADC_DIFFERENTIAL_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));

    // DMA konfiguracja
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)&ADC1->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, 2);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

    ADC1->CFGR &= ~ADC_CFGR_DMNGT;
    ADC1->CFGR |= (ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1);  // DMA circular

    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
    LL_ADC_REG_StartConversion(ADC1);
}

void ADC2_Init_Custom(uint16_t *buffer) {
    // Kalibracja w trybie różnicowym
    LL_ADC_StartCalibration(ADC2, LL_ADC_CALIB_OFFSET, LL_ADC_DIFFERENTIAL_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2));

    // DMA konfiguracja
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&ADC2->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, 2); // 2 kanały!
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

    ADC2->CFGR &= ~ADC_CFGR_DMNGT;
    ADC2->CFGR |= (ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1);  // DMA circular

    LL_ADC_Enable(ADC2);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2));
    LL_ADC_REG_StartConversion(ADC2);
}




float adc_to_voltage_out(float adc_value) {
    // Stałe z dopasowania liniowego
    const float a = 0.008793f;  // Nachylenie (V na jednostkę ADC)
    const float b = 0.017f;     // Przesunięcie (offset, V)

    // Przeliczenie wartości ADC na napięcie
    return a * adc_value ;
}



float adc_to_current_inductor(float adc_value) {
    const float a = 0.000500f;  // Nachylenie [A/LSB]
    const float b = -0.011f;    // Przesunięcie [A]
    return a * adc_value + b;
}


float adc_to_current_output(float adc_value) {
    const float a = 0.000500f;  // Nachylenie [A/LSB]
    const float b = -0.010f ;    // Przesunięcie [A]
    return a * adc_value + b;
}




