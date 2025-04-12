#ifndef BATTERY_SENSOR_H
#define BATTERY_SENSOR_H

#include <Arduino.h>
#include "config/pins_config.h"
#include "debug.h"

/**
 * @brief Clase para manejar la lectura del voltaje de la batería
 */
class BatterySensor {
public:
    /**
     * @brief Lee el voltaje de la batería
     * 
     * Activa el pin de control BATTERY_CONTROL_PIN para habilitar
     * la medición a través del divisor de voltaje y luego lo desactiva
     * para ahorrar energía.
     * 
     * @return float Voltaje de la batería en voltios, o NAN si hay error
     */
    static float readVoltage();

private:
    /**
     * @brief Calcula el voltaje real de la batería a partir de la lectura del ADC
     * 
     * El circuito es un divisor de voltaje:
     * 
     * Batería (+) ---- R2 (390k) ---- | ---- R1 (100k) ---- GND
     *                                 |
     *                                 +--- ADC Pin
     * 
     * La fórmula es:
     * VBAT = VADC_IN1 / (R1 / (R1 + R2))
     * 
     * @param adcVoltage Voltaje medido por el ADC
     * @return float Voltaje real de la batería
     */
    static float calculateBatteryVoltage(float adcVoltage);
};

#endif // BATTERY_SENSOR_H 