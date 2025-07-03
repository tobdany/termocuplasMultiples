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

//Estructura para cada sensor
Max31856_HandleTypeDef mySensor1;
Max31856_HandleTypeDef mySensor2;
Max31856_HandleTypeDef mySensor3;

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

const char MSG_INIT_ERROR[] = "Error: Could not initialize thermocouple\n\r";
// Note: We'll modify these prefixes slightly for better concatenation
const char MSG_THERMO_1_PREFIX[] = "Thermo 1: %.2f C\n\r";
const char MSG_THERMO_2_PREFIX[] = "Thermo 2: %.2f C\n\r";
const char MSG_THERMO_3_PREFIX[] = "Thermo 3: %.2f C\n\r";
const char MSG_LINE_SEPARATOR[] = "---------------------------\n\r"; // Optional separator

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
    HAL_Delay(20);

    while (1)
    {
        // A larger buffer is needed to hold all three messages plus potential separators
        // Max chars for one line (~30) * 3 lines + separator + null terminator.
        // 100 * 3 = 300 should be more than enough.
        char combined_buffer[300];
        int offset = 0; // To keep track of the current position in the combined_buffer

        // Format and append temperature for Sensor 1
        offset += sprintf(combined_buffer + offset, MSG_THERMO_1_PREFIX, MAX31856_ReadThermocoupleTemperature(&mySensor1));

        // Format and append temperature for Sensor 2
        offset += sprintf(combined_buffer + offset, MSG_THERMO_2_PREFIX, MAX31856_ReadThermocoupleTemperature(&mySensor2));

        // Format and append temperature for Sensor 3
        offset += sprintf(combined_buffer + offset, MSG_THERMO_3_PREFIX, MAX31856_ReadThermocoupleTemperature(&mySensor3));

        // Optional: Add a separator line at the end of each full transmission
        offset += sprintf(combined_buffer + offset, MSG_LINE_SEPARATOR);

        // Transmit the entire combined string at once
        HAL_UART_Transmit(&huart1, (uint8_t*)combined_buffer, strlen(combined_buffer), HAL_MAX_DELAY);

        HAL_Delay(10); // Increased delay for readability, adjust as needed
    }
}
