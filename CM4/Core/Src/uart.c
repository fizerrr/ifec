#include "uart.h"
#include <string.h>



char rxBuffer[RX_BUFFER_SIZE];


extern UART_HandleTypeDef huart4;


/**
 * @brief  Inicjalizuje dany UART z wybraną konfiguracją.
 * @param  huart: Wskaźnik na obiekt UART_HandleTypeDef.
 * @param  baudRate: Żądana prędkość transmisji UART (w bitach na sekundę).
 */
void UART_Init(UART_HandleTypeDef *huart, uint32_t baudRate)
{
    huart->Init.BaudRate = baudRate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(huart) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK)
    {
        Error_Handler();
    }


    HAL_NVIC_SetPriority(UART4_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

}

/**
 * @brief  Wysyła tekst przez wybrany UART.
 * @param  huart: Wskaźnik na obiekt UART_HandleTypeDef.
 * @param  string: Wskaźnik na bufor z tekstem do wysłania.
 */
void UART_Transmit(UART_HandleTypeDef *huart, char *string)
{
    HAL_UART_Transmit(huart, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}

/**
 * @brief  Odbiera dane przez wybrany UART.
 * @param  huart: Wskaźnik na obiekt UART_HandleTypeDef.
 * @param  buffer: Wskaźnik na bufor, gdzie zapisane zostaną odebrane dane.
 * @param  size: Liczba bajtów do odebrania.
 */
void UART_Receive(UART_HandleTypeDef *huart, char *buffer, uint16_t size)
{
    HAL_UART_Receive(huart, (uint8_t *)buffer, size, HAL_MAX_DELAY);
}

/**
 * @brief  Rozpoczyna odbiór danych z wykorzystaniem przerwań.
 * @param  huart: Wskaźnik na obiekt UART_HandleTypeDef.
 * @param  buffer: Wskaźnik na bufor, gdzie zapisane zostaną odebrane dane.
 * @param  size: Rozmiar bufora.
 */
void UART_Receive_IT(UART_HandleTypeDef *huart, char *buffer, uint16_t size)
{
    if (HAL_UART_Receive_IT(huart, (uint8_t *)buffer, size) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Callback wywoływany po zakończeniu odbioru danych.
 * @param  huart: Wskaźnik na obiekt UART_HandleTypeDef.
 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


    if (huart == &huart4)
    {

        UART_Transmit(&huart4, "Odebrano dane: ");
        UART_Transmit(&huart4, rxBuffer);
        UART_Transmit(&huart4, "\r\n");


        if (HAL_UART_Receive_IT(&huart4, (uint8_t *)rxBuffer, RX_BUFFER_SIZE) != HAL_OK)
        {
            UART_Transmit(&huart4, "Błąd przy ponownym uruchomieniu odbioru\r\n");
        }
    }
}


/**
 * @brief  Funkcja obsługi błędów UART.
 * @param  huart: Wskaźnik na obiekt UART_HandleTypeDef.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // Obsługa błędów UART
    // np. ponowne uruchomienie odbioru
}
