#include "sensors/BatterySensor.h"
// No necesita incluir config/pins_config.h ya que se incluye en el .h
#include "config/calibration_defaults.h" // Para las constantes BATTERY_R1 y BATTERY_R2

/**
 * @brief Lee el voltaje de la batería
 * 
 * @return float Voltaje de la batería en voltios, o NAN si hay error
 */
float BatterySensor::readVoltage() {
    // Activar el pin de control para habilitar la medición
    pinMode(BATTERY_CONTROL_PIN, OUTPUT);
    digitalWrite(BATTERY_CONTROL_PIN, LOW);
    
    // Esperar un breve momento para estabilizar la lectura
    delay(10);
    
    // Leer el valor del pin analógico para la batería en milivoltios directamente
    int milliVolts = analogReadMilliVolts(BATTERY_SENSOR_PIN);

    // Desactivar el pin de control para ahorrar energía
    digitalWrite(BATTERY_CONTROL_PIN, HIGH);
    
    // Convertir de milivoltios a voltios
    float voltage = milliVolts / 1000.0f;
    
    // Comprobar si el voltaje es válido
    if (isnan(voltage) || voltage <= 0.0f || voltage >= 3.3f) {
        return NAN;
    }

    // Calcular el voltaje real de la batería
    float batteryVoltage = calculateBatteryVoltage(voltage);
    return batteryVoltage;
}

/**
 * @brief Calcula el voltaje real de la batería a partir de la lectura del ADC
 * 
 * El circuito es un divisor de voltaje con:
 * R1 = 100k (a GND)
 * R2 = 390k (a batería)
 * VBAT = VADC_IN1 * (R1 + R2) / R1 = VADC_IN1 * (100k + 390k) / 100k
 * 
 * @param adcVoltage Voltaje medido por el ADC
 * @return float Voltaje real de la batería
 */
float BatterySensor::calculateBatteryVoltage(float adcVoltage) {
    // VBAT = 100k / (100k+390k) * VADC_IN1 corregido a:
    // VBAT = VADC_IN1 / (100k / (100k+390k))
    return adcVoltage / (BATTERY_R1 / (BATTERY_R1 + BATTERY_R2));
} 