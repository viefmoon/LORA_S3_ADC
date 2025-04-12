#include "sensors/CO2Sensor.h"

// Variable externa definida en main.cpp
extern SCD4x scd4x;

/**
 * @brief Inicializa el sensor SCD4x.
 */
bool CO2Sensor::begin() {
    return scd4x.begin(false, true, false);
}

/**
 * @brief Lee CO2, temperatura y humedad del sensor SCD4x en modo single-shot.
 * 
 * @param outCO2 Variable donde se almacenará la concentración de CO2 en ppm.
 * @param outTemp Variable donde se almacenará la temperatura en °C.
 * @param outHum Variable donde se almacenará la humedad relativa en %.
 * @return true si la lectura fue exitosa, false en caso contrario.
 */
bool CO2Sensor::read(float &outCO2, float &outTemp, float &outHum) {
    // Iniciar valores en NAN en caso de error
    outCO2 = NAN;
    outTemp = NAN;
    outHum = NAN;
    
    // Iniciar una medición en modo single-shot
    if (!scd4x.measureSingleShot()) {
        return false;
    }
    
    // Esperar hasta que haya datos disponibles (con timeout)
    uint32_t counter = 0;
    const uint32_t maxAttempts = 200; // 200 intentos * 50ms = 10 segundos máximo
    
    while (counter < maxAttempts) {
        delay(50); // Esperar 50ms entre comprobaciones
        counter++;
        
        if (scd4x.readMeasurement()) {
            // Leer los valores medidos
            outCO2 = (float)scd4x.getCO2();
            outTemp = scd4x.getTemperature();
            outHum = scd4x.getHumidity();
            return true;
        }
    }
    
    // Si llegamos aquí, hubo timeout
    return false;
} 