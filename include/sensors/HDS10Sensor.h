#ifndef HDS10_SENSOR_H
#define HDS10_SENSOR_H

#include <Arduino.h>
#include "config/pins_config.h"
#include "debug.h"

/**
 * @brief Clase para manejar el sensor de humedad HDS10
 */
class HDS10Sensor {
public:
    /**
     * @brief Lee el sensor HDS10 utilizando el ADS124S08
     * 
     * Realiza una medición diferencial entre AIN6 y AINCOM utilizando
     * el ADC externo ADS124S08.
     * 
     * @return float Porcentaje de humedad (0-100%) según calibración definida
     *               o NAN si ocurre un error o no es posible leer
     */
    static float read();

    /**
     * @brief Convierte la resistencia del sensor HDS10 a porcentaje de humedad
     * 
     * @param resistance Resistencia del sensor en ohms
     * @return float Porcentaje de humedad relativa (50-100%)
     */
    static float convertResistanceToHumidity(float resistance);
};

#endif // HDS10_SENSOR_H 