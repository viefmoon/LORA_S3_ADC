#ifndef VEML7700_SENSOR_H
#define VEML7700_SENSOR_H

#include <Arduino.h>
#include <Adafruit_VEML7700.h>

/**
 * @brief Clase para manejar el sensor de luz VEML7700
 */
class VEML7700Sensor {
public:
    /**
     * @brief Inicializa el sensor VEML7700 con configuración optimizada
     * para entornos de luz brillante y lectura rápida
     * 
     * @return true si la inicialización fue exitosa, false en caso contrario
     */
    static bool begin();
    
    /**
     * @brief Lee el valor de luz en lux del sensor VEML7700
     * 
     * @return float Valor en lux, o NAN si hay error
     */
    static float read();
};

#endif // VEML7700_SENSOR_H 