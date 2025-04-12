#ifndef DS18B20_SENSOR_H
#define DS18B20_SENSOR_H

#include <Arduino.h>
#include "config/pins_config.h"
#include "debug.h"

#include <DallasTemperature.h>

// Variable externa
extern DallasTemperature dallasTemp;

/**
 * @brief Clase para manejar el sensor de temperatura DS18B20
 */
class DS18B20Sensor {
public:
    /**
     * @brief Lee la temperatura del sensor DS18B20
     * 
     * @return float Temperatura en °C, o NAN si hay error
     */
    static float read();
};

#endif // DS18B20_SENSOR_H 