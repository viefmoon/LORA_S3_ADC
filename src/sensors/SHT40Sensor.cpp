#include "sensors/SHT40Sensor.h"

/**
 * @brief Inicializa el sensor SHT40
 * 
 * @return true si se inicializó correctamente, false en caso contrario
 */
bool SHT40Sensor::begin() {
    // Inicializar el objeto SHT40 con el bus I2C y la dirección
    sht40Sensor.begin(Wire, SHT40_I2C_ADDR_44);
    
    // Hacer un reset para asegurar que el sensor está en un estado limpio
    int16_t error = sht40Sensor.softReset();
    if (error != 0) {
        return false;
    }
    
    // Realizar una lectura de prueba
    float temp, hum;
    error = sht40Sensor.measureHighPrecision(temp, hum);
    if (error != 0) {
        return false;
    }
    
    return true;
}

/**
 * @brief Lee temperatura y humedad del sensor SHT40
 * 
 * @param outTemp Variable donde se almacenará la temperatura en °C
 * @param outHum Variable donde se almacenará la humedad relativa en %
 * @return true si la lectura fue exitosa, false en caso contrario
 */
bool SHT40Sensor::read(float &outTemp, float &outHum) {
    // Intentar hasta 3 veces obtener una lectura válida
    for (int i = 0; i < 3; i++) {
        int16_t error = sht40Sensor.measureHighPrecision(outTemp, outHum);
        
        if (error == 0) {
            // Verificar que los valores sean válidos (no cero y dentro de rangos razonables)
            if (!isnan(outTemp) && !isnan(outHum) && 
                outTemp > -40.0f && outTemp < 125.0f && 
                outHum >= 0.0f && outHum <= 100.0f) {
                return true; // Retornar inmediatamente con la primera lectura válida
            }
        }
        
        delay(5); // Pequeña pausa entre intentos
    }
    
    // Si no se encontró ninguna lectura válida
    outTemp = NAN;
    outHum = NAN;
    return false;
} 