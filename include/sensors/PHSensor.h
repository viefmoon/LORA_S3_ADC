#ifndef PH_SENSOR_H
#define PH_SENSOR_H

#include <Arduino.h>
#include "config/pins_config.h"
#include "debug.h"
#include "config_manager.h"

/**
 * @brief Clase para manejar el sensor de pH
 */
class PHSensor {
public:
    /**
     * @brief Lee el sensor de pH utilizando el ADS124S08
     * 
     * Realiza una medición diferencial entre AIN10 y AINCOM utilizando
     * el ADC externo ADS124S08.
     * 
     * @return float Valor de pH (0-14), o NAN si hay error
     */
    static float read();

    /**
     * @brief Convierte el voltaje medido a valor de pH
     * 
     * @param voltage Voltaje medido del sensor de pH
     * @param tempC Temperatura del agua en grados Celsius para compensación
     * @return float Valor de pH (0-14)
     */
    static float convertVoltageToPH(float voltage, float tempC);
};

#endif // PH_SENSOR_H 