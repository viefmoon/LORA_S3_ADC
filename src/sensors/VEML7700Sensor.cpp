#include "sensors/VEML7700Sensor.h"

// Variable externa del sensor VEML7700
extern Adafruit_VEML7700 veml7700;

/**
 * @brief Inicializa el sensor VEML7700 y lo configura para luz brillante/exterior
 * 
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool VEML7700Sensor::begin() {
    bool success = false;
    
    // Intentar inicializar hasta 3 veces si falla
    for (int i = 0; i < 3; i++) {
        success = veml7700.begin();
        if (success) break;
        delay(5); // Pequeña pausa entre intentos
    }
    
    if (!success) {
        return false;
    }
    
    // Configurar para entornos de luz brillante (exterior/solar)
    // Usar la ganancia más baja para manejar la alta intensidad del sol sin saturar
    veml7700.setGain(VEML7700_GAIN_1_8);
    
    // Usar el tiempo de integración más corto para una lectura rápida
    veml7700.setIntegrationTime(VEML7700_IT_25MS);
    
    return true;
}

/**
 * @brief Lee el nivel de luz en lux del sensor VEML7700
 * 
 * @return float Valor en lux, o NAN si hay error
 */
float VEML7700Sensor::read() {
    // Leer el valor de luz en lux utilizando la configuración fija (no el modo automático)
    // para maximizar la velocidad y minimizar el consumo de energía
    float lux = veml7700.readLux();
    
    // Verificar si la lectura es válida
    if (isnan(lux) || lux < 0.0f) {
        return NAN;
    }
    
    return lux;
} 