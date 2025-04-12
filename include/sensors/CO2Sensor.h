#ifndef CO2_SENSOR_H
#define CO2_SENSOR_H

#include <Arduino.h>
#include "config/pins_config.h"
#include "debug.h"
#include "SparkFun_SCD4x_Arduino_Library.h"

// Declaración externa del objeto del sensor (se definirá en src/main.cpp)
extern SCD4x scd4x;

/**
 * @brief Clase para manejar el sensor de CO2 SCD4x
 */
class CO2Sensor {
public:
    /**
     * @brief Lee CO2, temperatura y humedad del sensor SCD4x en modo single-shot.
     *
     * @param outCO2 Variable donde se almacenará la concentración de CO2 en ppm.
     * @param outTemp Variable donde se almacenará la temperatura en °C.
     * @param outHum Variable donde se almacenará la humedad relativa en %.
     * @return true si la lectura fue exitosa, false en caso contrario.
     */
    static bool read(float &outCO2, float &outTemp, float &outHum);

    /**
     * @brief Inicializa el sensor SCD4x.
     * @return true si la inicialización fue exitosa, false en caso contrario.
     */
    static bool begin();
};

#endif // CO2_SENSOR_H 