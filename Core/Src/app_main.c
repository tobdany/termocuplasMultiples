#include "app_main.h"
#include "Max31856.h"
#include <stdio.h>
#include <string.h>

// Define los pines CS para cada sensor
#define SENSOR1_CS_PORT GPIOA
#define SENSOR1_CS_PIN  GPIO_PIN_4

#define SENSOR2_CS_PORT GPIOA
#define SENSOR2_CS_PIN  GPIO_PIN_0

#define SENSOR3_CS_PORT GPIOA
#define SENSOR3_CS_PIN  GPIO_PIN_1

// Estructura para cada sensor
Max31856_HandleTypeDef mySensor1;
Max31856_HandleTypeDef mySensor2;
Max31856_HandleTypeDef mySensor3;

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

const char MSG_INIT_ERROR[] = "Error: Could not initialize thermocouple\n\r";
const char MSG_THERMO_1_PREFIX[] = "Thermo 1: %.2f C\n\r";
const char MSG_THERMO_2_PREFIX[] = "Thermo 2: %.2f C\n\r";
const char MSG_THERMO_3_PREFIX[] = "Thermo 3: %.2f C\n\r";
const char MSG_LINE_SEPARATOR[] = "---------------------------\n\r"; // Optional separator


#define MAX_TEMP_LINE_LENGTH (25)

// Calcular el tamaño total del buffer
#define TOTAL_MESSAGE_BUFFER_SIZE (MAX_TEMP_LINE_LENGTH * 3 + strlen(MSG_LINE_SEPARATOR) + 1)


/**
  * @brief  Función principal de la aplicación.
  * Contiene la lógica de inicialización de sensores y el bucle infinito.
  * @retval None
  */
void app_main(void)
{
    // Check sensor initializations
    if (!MAX31856_Init(&mySensor1, &hspi1, SENSOR1_CS_PORT, SENSOR1_CS_PIN)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)MSG_INIT_ERROR, strlen(MSG_INIT_ERROR), HAL_MAX_DELAY);
        while (1) HAL_Delay(10);
    }
    if (!MAX31856_Init(&mySensor2, &hspi1, SENSOR2_CS_PORT, SENSOR2_CS_PIN)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)MSG_INIT_ERROR, strlen(MSG_INIT_ERROR), HAL_MAX_DELAY);
        while (1) HAL_Delay(10);
    }
    if (!MAX31856_Init(&mySensor3, &hspi1, SENSOR3_CS_PORT, SENSOR3_CS_PIN)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)MSG_INIT_ERROR, strlen(MSG_INIT_ERROR), HAL_MAX_DELAY);
        while (1) HAL_Delay(10);
    }
    HAL_Delay(200);

    while (1)
    {

    	char combined_buffer[TOTAL_MESSAGE_BUFFER_SIZE];
        int offset = 0;
        offset += sprintf(combined_buffer + offset, MSG_THERMO_1_PREFIX, MAX31856_ReadThermocoupleTemperature(&mySensor1));
        offset += sprintf(combined_buffer + offset, MSG_THERMO_2_PREFIX, MAX31856_ReadThermocoupleTemperature(&mySensor2));
        offset += sprintf(combined_buffer + offset, MSG_THERMO_3_PREFIX, MAX31856_ReadThermocoupleTemperature(&mySensor3));
        offset += sprintf(combined_buffer + offset, MSG_LINE_SEPARATOR);
        HAL_UART_Transmit(&huart1, (uint8_t*)combined_buffer, strlen(combined_buffer), HAL_MAX_DELAY);

        HAL_Delay(20);
    }
}
