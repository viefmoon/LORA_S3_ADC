#include "sensors/PHSensor.h"

#include <cmath>
#include "sensors/NtcManager.h"
#include "config/pins_config.h"
#include "config_manager.h"
#include "AdcUtilities.h"
#include "ADS124S08.h"

/**
 * @brief Convierte el voltaje medido a valor de pH
 * 
 * @param voltage Voltaje medido del sensor de pH
 * @param tempC Temperatura del agua en grados Celsius para compensación
 * @return float Valor de pH (0-14)
 */
float PHSensor::convertVoltageToPH(float voltage, float tempC) {
    float V1, T1, V2, T2, V3, T3, TEMP_CAL;
    ConfigManager::getPHConfig(V1, T1, V2, T2, V3, T3, TEMP_CAL);

    // Si solutionTemp es NAN, usar la temperatura de calibración como valor por defecto
    if (isnan(tempC)) {
        tempC = TEMP_CAL;
    }

    // Datos de calibración (pH, voltaje)
    const double pH_calib[] = {T1, T2, T3};
    const double V_calib[] = {V1, V2, V3};
    const int n = 3; // Número de puntos de calibración

    // Calcular sumatorias necesarias para mínimos cuadrados
    double sum_pH = 0.0;
    double sum_V = 0.0;
    double sum_pHV = 0.0;
    double sum_pH2 = 0.0;

    for (int i = 0; i < n; i++) {
        sum_pH += pH_calib[i];
        sum_V += V_calib[i];
        sum_pHV += pH_calib[i] * V_calib[i];
        sum_pH2 += pH_calib[i] * pH_calib[i];
    }

    // Calcular la pendiente S usando mínimos cuadrados
    double S_CAL = ((n * sum_pHV) - (sum_pH * sum_V)) / ((n * sum_pH2) - (sum_pH * sum_pH));

    // Calcular el offset E0 usando mínimos cuadrados
    double E0 = ((sum_V) + (S_CAL * sum_pH)) / n;

    // Ajustar la pendiente según la temperatura actual usando la ecuación de Nernst
    const double tempK = (tempC + 273.15);
    const double tempCalK = (TEMP_CAL + 273.15);
    const double S_T = S_CAL * (tempK / tempCalK);

    // Calcular pH usando la ecuación de Nernst ajustada: pH = (E0 - E) / S(T)
    double pH = ((E0 + voltage) / S_T);
    // Limitar el pH a un rango físicamente posible (0-14)
    pH = constrain(pH, 0.0, 14.0);

    return pH;
}

/**
 * @brief Lee el sensor de pH usando el ADC externo ADS124S08
 * 
 * @return float Valor de pH (0-14), o NAN si hay error
 */
float PHSensor::read() {
    // Configurar para medir diferencial entre AIN10 y AINCOM
    uint8_t muxConfig = ADS_P_AIN10 | ADS_N_AINCOM;
    
    // Realizar la medición diferencial utilizando el ADC externo
    float voltage = AdcUtilities::measureAdcDifferential(muxConfig);
    
    // Verificar si el voltaje es válido (0-2.5V para el ADS124S08)
    if (isnan(voltage) || voltage < 0.0f || voltage > 2.5f) {
        return NAN;
    }
    
    
    // Obtener temperatura únicamente del sensor NTC10K
    float waterTemp = NtcManager::readNtc10kTemperature();
    
    // Convertir a pH con compensación de temperatura
    float pHValue = convertVoltageToPH(voltage, waterTemp);
    
    return pHValue;
} 