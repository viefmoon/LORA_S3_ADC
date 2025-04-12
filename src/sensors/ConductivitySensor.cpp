#include "sensors/ConductivitySensor.h"

#include <cmath>
#include "sensors/NtcManager.h"
#include "config/pins_config.h"
#include "config_manager.h"
#include "AdcUtilities.h"
#include "ADS124S08.h"

/**
 * @brief Convierte el voltaje medido a valor de conductividad/TDS en ppm
 * 
 * @param voltage Voltaje medido del sensor
 * @param tempC Temperatura del agua en grados Celsius para compensación
 * @return float Valor de TDS en ppm (partes por millón)
 */
float ConductivitySensor::convertVoltageToConductivity(float voltage, float tempC) {
    float calTemp, coefComp, V1, T1, V2, T2, V3, T3;
    ConfigManager::getConductivityConfig(calTemp, coefComp, V1, T1, V2, T2, V3, T3);

    // Si tempC es NAN, usar la temperatura de calibración como valor por defecto
    if (isnan(tempC)) {
        tempC = calTemp;
    }

    // Matriz para resolver el sistema de ecuaciones
    // Basado en 3 puntos de calibración
    const double det = V1*V1*(V2 - V3) - V1*(V2*V2 - V3*V3) + (V2*V2*V3 - V2*V3*V3);
    
    // Calcular coeficientes solo si el determinante no es cero
    if(fabs(det) > 1e-6) {
        const double a = (T1*(V2 - V3) - T2*(V1 - V3) + T3*(V1 - V2)) / det;
        const double b = (T1*(V3*V3 - V2*V2) + T2*(V1*V1 - V3*V3) + T3*(V2*V2 - V1*V1)) / det;
        const double c = (T1*(V2*V2*V3 - V2*V3*V3) - T2*(V1*V1*V3 - V1*V3*V3) + T3*(V1*V1*V2 - V1*V2*V2)) / det;
        
        // Aplicar compensación de temperatura
        const double compensation = 1.0 + coefComp * (tempC - calTemp);
        double compensatedVoltage = voltage / compensation;
        double conductivity = a * (compensatedVoltage * compensatedVoltage) 
                    + b * compensatedVoltage 
                    + c;

        return fmax(conductivity, 0.0);
    }
    else {
        return NAN;
    }
}

/**
 * @brief Lee el sensor de conductividad usando el ADC externo ADS124S08
 * 
 * @return float Valor de conductividad/TDS en ppm, o NAN si hay error
 */
float ConductivitySensor::read() {
    // Configurar para medir diferencial entre AIN9 y AINCOM
    uint8_t muxConfig = ADS_P_AIN9 | ADS_N_AINCOM;
    
    // Realizar la medición diferencial utilizando el ADC externo
    float voltage = AdcUtilities::measureAdcDifferential(muxConfig);
    
    // Verificar si el voltaje está en rango válido (0-2.5V para el ADS124S08)
    if (isnan(voltage) || voltage < 0.0f || voltage > 2.5f) {
        return NAN; // Valor fuera de rango
    }
    
    // Obtener temperatura únicamente del sensor NTC10K
    float waterTemp = NtcManager::readNtc10kTemperature();
    
    // Convertir a conductividad con compensación de temperatura
    float tdsValue = convertVoltageToConductivity(voltage, waterTemp);
    return tdsValue;
} 