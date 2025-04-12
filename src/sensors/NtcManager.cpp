#include "sensors/NtcManager.h"
#include <cmath>  // Para fabs() y otras funciones matemáticas
#include "config_manager.h"
#include "debug.h"
#include "config/sensor_defaults.h"  // Para NTC_TEMP_MIN y NTC_TEMP_MAX
#include "config/pins_config.h"      // Para NTC100K_0_PIN, NTC100K_1_PIN y NTC10K_PIN
#include "AdcUtilities.h"            // Para usar el ADS124S08
#include "ADS124S08.h"               // Para las definiciones de registros

void NtcManager::calculateSteinhartHartCoeffs(double T1, double R1,
                                          double T2, double R2,
                                          double T3, double R3,
                                          double &A, double &B, double &C) 
{
    // Ecuación de Steinhart-Hart:
    // 1/T = A + B*ln(R) + C*(ln(R))^3
    
    double L1 = log(R1);
    double L2 = log(R2);
    double L3 = log(R3);
    
    double Y1 = 1.0 / T1;
    double Y2 = 1.0 / T2;
    double Y3 = 1.0 / T3;
    
    // Resolver sistema de ecuaciones para encontrar A, B, C
    double L1_3 = L1 * L1 * L1;
    double L2_3 = L2 * L2 * L2;
    double L3_3 = L3 * L3 * L3;
    
    double denominator = (L2 - L1) * (L3 - L1) * (L3 - L2);
    
    // Protección contra división por cero
    if (fabs(denominator) < 1e-10) {
        // Asignar NAN para indicar claramente el error
        A = NAN;
        B = NAN;
        C = NAN;
        return;
    }
    
    // Calcular C
    C = ((Y2 - Y1) * (L3 - L1) - (Y3 - Y1) * (L2 - L1)) / 
        ((L2_3 - L1_3) * (L3 - L1) - (L3_3 - L1_3) * (L2 - L1));
    
    // Calcular B
    B = ((Y2 - Y1) - C * (L2_3 - L1_3)) / (L2 - L1);
    
    // Calcular A
    A = Y1 - B * L1 - C * L1_3;
}

double NtcManager::steinhartHartTemperature(double resistance, double A, double B, double C) 
{
    if (resistance <= 0.0) {
        return NAN;
    }
    double lnR = log(resistance);
    double invT = A + B * lnR + C * lnR*lnR*lnR;  // 1/T en Kelvin^-1
    double tempK = 1.0 / invT;                   // Kelvin
    double tempC = tempK - 273.15;               // °C
    return tempC;
}

double NtcManager::computeNtcResistanceFromVoltageDivider(double voltage, double vRef, double rFixed, bool ntcTop)
{
    // Validación de rangos
    if (voltage <= 0.0 || voltage >= vRef) {
        return -1.0;  // Indica valor inválido
    }
    
    // Calcular resistencia del NTC
    double Rntc;
    
    if (ntcTop) {
        // NTC conectado a Vref (arriba) y resistencia fija a GND (abajo)
        // Fórmula: Rntc = rFixed * (vRef - voltage) / voltage
        Rntc = rFixed * ((vRef - voltage) / voltage);
    } else {
        // NTC conectado a GND (abajo) y resistencia fija a Vref (arriba)
        // Fórmula: Rntc = rFixed * voltage / (vRef - voltage)
        Rntc = rFixed * (voltage / (vRef - voltage));
    }
    
    return Rntc;
}

double NtcManager::readNtc100kTemperature(const char* configKey) {
    // Obtener calibración NTC100K de la configuración
    double t1=25.0, r1=100000.0, t2=35.0, r2=64770.0, t3=45.0, r3=42530.0;
    ConfigManager::getNTC100KConfig(t1, r1, t2, r2, t3, r3);

    // Pasar °C a Kelvin
    double T1K = t1 + 273.15;
    double T2K = t2 + 273.15;
    double T3K = t3 + 273.15;

    // Calcular coeficientes Steinhart-Hart
    double A=0, B=0, C=0;
    calculateSteinhartHartCoeffs(T1K, r1, T2K, r2, T3K, r3, A, B, C);
    
    // Configurar el mux según el sensor
    uint8_t muxConfig = 0;
    if (strcmp(configKey, "0") == 0) {
        // NTC100K_0: AIN1 (positivo) y AIN0 (negativo)
        muxConfig = ADS_P_AIN1 | ADS_N_AIN0;
    } else if (strcmp(configKey, "1") == 0) {
        // NTC100K_1: AIN3 (positivo) y AIN2 (negativo)
        muxConfig = ADS_P_AIN3 | ADS_N_AIN2;
    } else {
        // Si no coincide con ninguna configuración, retornamos NAN
        return NAN;
    }

    // Realizar la medición diferencial utilizando el ADC externo
    float voltage = AdcUtilities::measureAdcDifferential(muxConfig);
    
    // Verificar si el voltaje es válido (debería estar en rango ±2.5V)
    if (isnan(voltage) || voltage <= -2.5f || voltage >= 2.5f) {
        return NAN;
    }

    // Calcular la resistencia del NTC a partir del voltaje diferencial del puente
    double Rntc = computeNtcResistanceFromBridge(voltage);
    if (Rntc <= 0.0) {
        return NAN;
    }

    // Usar Steinhart-Hart para calcular la temperatura en °C
    double tempC = steinhartHartTemperature(Rntc, A, B, C);
    
    // Validar que el valor de temperatura está dentro de los límites aceptables
    if (isnan(tempC) || tempC < NTC_TEMP_MIN || tempC > NTC_TEMP_MAX) {
        return NAN;
    }
    
    return tempC;
}

double NtcManager::readNtc10kTemperature() {
    // Obtener calibración NTC10K de la configuración
    // Usando valores por defecto para un NTC10K común
    double t1=25.0, r1=10000.0, t2=50.0, r2=3893.0, t3=85.0, r3=1218.0;
    ConfigManager::getNTC10KConfig(t1, r1, t2, r2, t3, r3);

    // Pasar °C a Kelvin
    double T1K = t1 + 273.15;
    double T2K = t2 + 273.15;
    double T3K = t3 + 273.15;

    // Calcular coeficientes Steinhart-Hart
    double A=0, B=0, C=0;
    calculateSteinhartHartCoeffs(T1K, r1, T2K, r2, T3K, r3, A, B, C);

    // Configurar para medir entre AIN4 (positivo) y AINCOM (negativo)
    uint8_t muxConfig = ADS_P_AIN4 | ADS_N_AINCOM;
    
    // Realizar la medición diferencial utilizando el ADC externo
    float voltage = AdcUtilities::measureAdcDifferential(muxConfig);
    
    // Verificar si el voltaje es válido (0-2.5V para divisor de tensión)
    if (isnan(voltage) || voltage < 0.0f || voltage > 2.5f) {
        return NAN;
    }

    // El NTC10K está conectado en un divisor de tensión:
    // 2.5V --- NTC10K --- [PUNTO DE MEDICIÓN] --- 10K --- GND
    // Donde el punto de medición es AIN4
    double vRef = 2.5; // Voltaje de referencia (2.5V)
    double rFixed = 10000.0; // Resistencia fija (10k)
    bool ntcTop = true; // NTC está conectado a Vref (arriba)
    
    // Calcular la resistencia NTC a partir del voltaje del divisor
    double Rntc = computeNtcResistanceFromVoltageDivider(voltage, vRef, rFixed, ntcTop);
    if (Rntc <= 0.0) {
        return NAN;
    }

    // Usar Steinhart-Hart para calcular la temperatura en °C
    double tempC = steinhartHartTemperature(Rntc, A, B, C);
    
    // Validar que el valor de temperatura está dentro de los límites aceptables
    if (isnan(tempC) || tempC < NTC_TEMP_MIN || tempC > NTC_TEMP_MAX) {
        return NAN;
    }
    
    return tempC;
}

/**
 * @brief Calcula la resistencia del NTC en un puente de Wheatstone a partir del voltaje diferencial
 * @param diffVoltage Voltaje diferencial medido (V)
 * @return Resistencia del NTC (ohms) o -1 si hay error
 * 
 * El puente Wheatstone se modela como:
 * 2.5V --- R1 --- [POS] --- NTC --- GND
 *      |                |
 *      --- R3 --- [NEG] --- R2 --- GND
 * 
 * Donde R1 = R2 = R3 = 100k y NTC es la resistencia a medir
 */
double NtcManager::computeNtcResistanceFromBridge(double diffVoltage)
{
    // Resistencias fijas del puente
    const double R1 = 100000.0; // 100k
    const double R2 = 100000.0; // 100k
    const double R3 = 100000.0; // 100k
    const double vRef = 2.5;    // Voltaje de referencia 2.5V
    
    // En un puente equilibrado, diffVoltage = 0V cuando NTC = R2
    // Con temperatura variable, diffVoltage != 0
    
    // Si diffVoltage es muy cercano a vRef o -vRef, hay un problema con el circuito
    if (fabs(diffVoltage) >= vRef * 0.9) {
        return -1.0;
    }
    
    // Calcular R4 (NTC) a partir de la ecuación del puente:
    // diffVoltage = vRef * (R2/(R2+NTC) - R3/(R1+R3))
    // Despejando NTC:
    double ratio = (diffVoltage / vRef + R3 / (R1 + R3)) / (1 - diffVoltage / vRef);
    double ntcR = R2 / ratio - R2;
    
    // Validar el resultado
    if (ntcR <= 0.0) {
        return -1.0;
    }
    
    return ntcR;
} 