#ifndef SHT40_SENSOR_H
#define SHT40_SENSOR_H

#include <Arduino.h>
#include "config/pins_config.h"
#include "debug.h"
#include <SensirionI2cSht4x.h>

// Variable externa del sensor SHT40 que será declarada en otro lugar
extern SensirionI2cSht4x sht40Sensor;

/**
 * @brief Clase para manejar el sensor de temperatura y humedad SHT40
 */
class SHT40Sensor {
public:
    /**
     * @brief Inicializa el sensor SHT40
     * 
     * @return true si se inicializó correctamente, false en caso contrario
     */
    static bool begin();

    /**
     * @brief Lee temperatura y humedad del sensor SHT40
     * 
     * @param outTemp Variable donde se almacenará la temperatura en °C
     * @param outHum Variable donde se almacenará la humedad relativa en %
     * @return true si la lectura fue exitosa, false en caso contrario
     */
    static bool read(float &outTemp, float &outHum);
};

#endif // SHT40_SENSOR_H 