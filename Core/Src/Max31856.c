#include "Max31856.h"
#include <math.h>
#include <stdio.h>


// --- Funciones internas de bajo nivel para comunicación SPI ---
bool MAX31856_Init(Max31856_HandleTypeDef* hmax, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin) {
  hmax->hspi = hspi;
  hmax->cs_port = cs_port;
  hmax->cs_pin = cs_pin;

  HAL_GPIO_WritePin(hmax->cs_port, hmax->cs_pin, GPIO_PIN_SET);
  hmax->initialized = true;

  // Configuración inicial del sensor MAX31856
  // Desactiva la máscara de fallas (assert on any fault)
  writeRegister8(hmax, MAX31856_MASK_REG, 0x0);

  // Habilita la detección de fallas de circuito abierto
  writeRegister8(hmax, MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);

  // Establece el offset de temperatura de la unión fría a cero
  writeRegister8(hmax, MAX31856_CJTO_REG, 0x0);

  MAX31856_SetThermocoupleType(hmax, MAX31856_TCTYPE_K);
  MAX31856_SetConversionMode(hmax, MAX31856_ONESHOT);

  return hmax->initialized;
}

/**
  * @brief  Lee N bytes de un registro del MAX31856.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  addr: Dirección del registro a leer (MSB=0 para lectura).
  * @param  buffer: Puntero al buffer donde se almacenarán los datos leídos.
  * @param  n: Número de bytes a leer.
  * @retval None
  */
void readRegisterN(Max31856_HandleTypeDef* hmax, uint8_t addr, uint8_t buffer[], uint8_t n) {
  addr &= 0x7F; // MSB=0 para lectura, asegura que el bit superior no esté seteado


  HAL_GPIO_WritePin(hmax->cs_port, hmax->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hmax->hspi, &addr, 1, SPI_DELAY);
  HAL_SPI_Receive(hmax->hspi, buffer, n, SPI_DELAY);
  HAL_GPIO_WritePin(hmax->cs_port, hmax->cs_pin, GPIO_PIN_SET);
}

/**
  * @brief  Escribe un byte en un registro del MAX31856.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  addr: Dirección del registro a escribir (MSB=1 para escritura).
  * @param  data: Byte de datos a escribir.
  * @retval None
  */
void writeRegister8(Max31856_HandleTypeDef* hmax, uint8_t addr, uint8_t data) {
  addr |= 0x80; // MSB=1 para escritura, asegura que el bit superior esté seteado

  uint8_t buffer[2] = {addr, data};

  // Baja el pin CS del sensor específico para seleccionarlo
  HAL_GPIO_WritePin(hmax->cs_port, hmax->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hmax->hspi, buffer, 2, SPI_DELAY);
  HAL_GPIO_WritePin(hmax->cs_port, hmax->cs_pin, GPIO_PIN_SET);
}

// --- Funciones de lectura de registros con tipos de datos específicos ---

/**
  * @brief  Lee un byte de un registro del MAX31856.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  addr: Dirección del registro a leer.
  * @retval El byte leído del registro.
  */
uint8_t readRegister8(Max31856_HandleTypeDef* hmax, uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(hmax, addr, &ret, 1);
  return ret;
}

/**
  * @brief  Lee 16 bits (2 bytes) de un registro del MAX31856.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  addr: Dirección del registro a leer.
  * @retval Los 16 bits leídos del registro.
  */
uint16_t readRegister16(Max31856_HandleTypeDef* hmax, uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(hmax, addr, buffer, 2);

  uint16_t ret = (uint16_t)buffer[0]; // Castea a uint16_t antes del desplazamiento
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

/**
  * @brief  Lee 24 bits (3 bytes) de un registro del MAX31856.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  addr: Dirección del registro a leer.
  * @retval Los 24 bits leídos del registro.
  */
uint32_t readRegister24(Max31856_HandleTypeDef* hmax, uint8_t addr) {
  uint8_t buffer[3] = {0, 0, 0};
  readRegisterN(hmax, addr, buffer, 3);

  uint32_t ret = (uint32_t)buffer[0];
  ret <<= 8;
  ret |= buffer[1];
  ret <<= 8;
  ret |= buffer[2];

  return ret;
}

// --- Funciones de la API de la librería MAX31856 ---

/**
  * @brief  Inicializa el sensor MAX31856.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  hspi: Puntero al handle de la interfaz SPI (ej. &hspi1).
  * @param  cs_port: Puerto GPIO del pin Chip Select (ej. GPIOA).
  * @param  cs_pin: Pin GPIO del Chip Select (ej. SPI1_CS_Pin).
  * @retval true si la inicialización fue exitosa, false en caso contrario.
  */

/**
  * @brief  Obtiene el modo de conversión actual del sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval El modo de conversión (MAX31856_ONESHOT o MAX31856_CONTINUOUS).
  */
max31856_conversion_mode_t MAX31856_GetConversionMode(Max31856_HandleTypeDef* hmax) {
  return hmax->conversionMode;
}

/**
  * @brief  Establece el modo de conversión del sensor (One-Shot o Continuo).
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  mode: Modo de conversión a establecer.
  * @retval None
  */
void MAX31856_SetConversionMode(Max31856_HandleTypeDef* hmax, max31856_conversion_mode_t mode) {
  hmax->conversionMode = mode;
  uint8_t t = readRegister8(hmax, MAX31856_CR0_REG); // Obtiene el valor actual del registro
  if (hmax->conversionMode == MAX31856_CONTINUOUS) {
    t |= MAX31856_CR0_AUTOCONVERT; // Enciende la conversión automática
    t &= ~MAX31856_CR0_1SHOT;      // Apaga el modo one-shot
  } else {
    t &= ~MAX31856_CR0_AUTOCONVERT; // Apaga la conversión automática
    t |= MAX31856_CR0_1SHOT;        // Enciende el modo one-shot
  }
  writeRegister8(hmax, MAX31856_CR0_REG, t); // Escribe el valor de vuelta al registro
}

/**
  * @brief  Establece el tipo de termopar para el sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  type: Tipo de termopar a establecer.
  * @retval None
  */
void MAX31856_SetThermocoupleType(Max31856_HandleTypeDef* hmax, max31856_thermocoupletype_t type) {
  uint8_t t = readRegister8(hmax, MAX31856_CR1_REG);
  t &= 0xF0; // Enmascara los 4 bits inferiores
  t |= (uint8_t)type & 0x0F;
  writeRegister8(hmax, MAX31856_CR1_REG, t);
}

/**
  * @brief  Obtiene el tipo de termopar configurado en el sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval El tipo de termopar configurado.
  */
max31856_thermocoupletype_t MAX31856_GetThermocoupleType(Max31856_HandleTypeDef* hmax) {
  uint8_t t = readRegister8(hmax, MAX31856_CR1_REG);
  t &= 0x0F;

  return (max31856_thermocoupletype_t)(t);
}

/** lol
  * @brief  Lee el registro de fallas del sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval El valor del registro de estado de fallas.
  */
uint8_t MAX31856_ReadFault(Max31856_HandleTypeDef* hmax) {
  return readRegister8(hmax, MAX31856_SR_REG);
}

/**
  * @brief  Establece los umbrales de falla para la unión fría.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  low: Umbral inferior de temperatura para la unión fría (en °C).
  * @param  high: Umbral superior de temperatura para la unión fría (en °C).
  * @retval None
  */
void MAX31856_SetColdJunctionFaultThreshholds(Max31856_HandleTypeDef* hmax, int8_t low, int8_t high) {
  writeRegister8(hmax, MAX31856_CJLF_REG, low);
  writeRegister8(hmax, MAX31856_CJHF_REG, high);
}

/**
  * @brief  Establece el filtro de ruido (50Hz o 60Hz).
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  noiseFilter: Tipo de filtro de ruido a establecer.
  * @retval None
  */
void MAX31856_SetNoiseFilter(Max31856_HandleTypeDef* hmax, max31856_noise_filter_t noiseFilter) {
  uint8_t t = readRegister8(hmax, MAX31856_CR0_REG);
  if (noiseFilter == MAX31856_NOISE_FILTER_50HZ) {
    t |= 0x01; // Bit 0 para 50Hz
  } else {
    t &= 0xfe; // Bit 0 para 60Hz
  }
  writeRegister8(hmax, MAX31856_CR0_REG, t);
}

/**
  * @brief  Establece los umbrales de falla de temperatura del termopar.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @param  flow: Umbral inferior de temperatura del termopar (en °C).
  * @param  fhigh: Umbral superior de temperatura del termopar (en °C).
  * @retval None
  */
void MAX31856_SetTempFaultThreshholds(Max31856_HandleTypeDef* hmax, float flow, float fhigh) {
  int16_t low, high;

  // El MAX31856 almacena los umbrales como valores de 16 bits con una resolución de 0.0078125 °C/LSB.
  // Multiplicamos por 16 para convertir a la escala interna (1 LSB = 0.0625 °C, que es 1/16).
  // Los valores de 16 bits se dividen en dos registros de 8 bits (High y Low).
  flow *= 16;
  low = (int16_t)flow;

  fhigh *= 16;
  high = (int16_t)fhigh;

  // Umbral superior
  writeRegister8(hmax, MAX31856_LTHFTH_REG, (uint8_t)(high >> 8)); // MSB
  writeRegister8(hmax, MAX31856_LTHFTL_REG, (uint8_t)high);        // LSB

  // Umbral inferior
  writeRegister8(hmax, MAX31856_LTLFTH_REG, (uint8_t)(low >> 8));  // MSB
  writeRegister8(hmax, MAX31856_LTLFTL_REG, (uint8_t)low);         // LSB
}

/**
  * @brief  Dispara una conversión One-Shot en el sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval None
  */
void MAX31856_TriggerOneShot(Max31856_HandleTypeDef* hmax) {
  if (hmax->conversionMode == MAX31856_CONTINUOUS)
    return; // No se dispara one-shot si está en modo continuo

  uint8_t t = readRegister8(hmax, MAX31856_CR0_REG); // Obtiene el valor actual del registro
  t &= ~MAX31856_CR0_AUTOCONVERT;              // Apaga la conversión automática
  t |= MAX31856_CR0_1SHOT;                     // Enciende el modo one-shot
  writeRegister8(hmax, MAX31856_CR0_REG, t);   // Escribe el valor de vuelta al registro
                                       // La conversión comienza cuando CS se pone en alto (después de la escritura)
}

/**
  * @brief  Verifica si la conversión de temperatura ha sido completada.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval true si la conversión está completa, false en caso contrario.
  */
bool MAX31856_ConversionComplete(Max31856_HandleTypeDef* hmax) {
  if (hmax->conversionMode == MAX31856_CONTINUOUS)
    return true; // Siempre "completa" en modo continuo

  // La conversión está completa cuando el bit MAX31856_CR0_1SHOT se borra automáticamente
  return !(readRegister8(hmax, MAX31856_CR0_REG) & MAX31856_CR0_1SHOT);
}

/**
  * @brief  Lee la temperatura de la unión fría del sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval La temperatura de la unión fría en grados Celsius.
  */
float MAX31856_ReadCJTemperature(Max31856_HandleTypeDef* hmax) {
  // La temperatura de la unión fría se almacena en un registro de 16 bits,
  // con una resolución de 0.00390625 °C/LSB (1/256).
  return readRegister16(hmax, MAX31856_CJTH_REG) / 256.0;
}

/**
  * @brief  Lee la temperatura del termopar del sensor.
  * @param  hmax: Puntero a la estructura de handle del sensor MAX31856.
  * @retval La temperatura del termopar en grados Celsius, o NAN si hay timeout.
  */
float MAX31856_ReadThermocoupleTemperature(Max31856_HandleTypeDef* hmax) {
  // Para el modo One-Shot, dispara una conversión y espera a que termine
  if (hmax->conversionMode == MAX31856_ONESHOT) {
    MAX31856_TriggerOneShot(hmax);
    uint32_t start = HAL_GetTick();
    while (!MAX31856_ConversionComplete(hmax)) {
      if (HAL_GetTick() - start > 250) // Timeout de 250ms
        return NAN; // Retorna Not-A-Number en caso de timeout
      HAL_Delay(10); // Pequeño retraso para no saturar el bus
    }
  }

  // Lee los registros de temperatura linealizada del termopar (3 bytes)
  int32_t temp24 = (int32_t)readRegister24(hmax, MAX31856_LTCBH_REG); // Castea a int32_t para manejo de signo

  // El bit 23 es el bit de signo (MSB de los 24 bits de datos).
  // Si está seteado, el número es negativo y necesitamos extender el signo a 32 bits.
  if (temp24 & 0x800000) {
    temp24 |= 0xFF000000; // Extiende el signo para valores negativos
  }

  // Los 5 bits menos significativos (bits 0-4) no se usan y deben ser descartados.
  temp24 >>= 5;

  // La resolución es de 0.0078125 °C/LSB (1/128).
  return temp24 * 0.0078125;
}
