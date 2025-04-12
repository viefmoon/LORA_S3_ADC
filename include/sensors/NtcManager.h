#ifndef NTC_MANAGER_H
#define NTC_MANAGER_H

#include <Arduino.h>

/**
 * @brief Clase para gestionar los cálculos y lecturas de sensores NTC100K
 */
class NtcManager {
public:
    /**
     * @brief Calcula los coeficientes A, B y C de Steinhart-Hart usando 3 puntos (T, R).
     *        T debe estar en Kelvin. R en ohms.
     * @param T1 Temperatura en Kelvin en punto 1
     * @param R1 Resistencia en ohms en punto 1
     * @param T2 Temperatura en Kelvin en punto 2
     * @param R2 Resistencia en ohms en punto 2
     * @param T3 Temperatura en Kelvin en punto 3
     * @param R3 Resistencia en ohms en punto 3
     * @param A [out] Coeficiente A
     * @param B [out] Coeficiente B
     * @param C [out] Coeficiente C
     */
    static void calculateSteinhartHartCoeffs(double T1, double R1,
                                           double T2, double R2,
                                           double T3, double R3,
                                           double &A, double &B, double &C);

    /**
     * @brief Calcula la temperatura (°C) a partir de la resistencia usando la ecuación de Steinhart-Hart.
     * @param resistance Resistencia medida (ohms)
     * @param A Coeficiente A
     * @param B Coeficiente B
     * @param C Coeficiente C
     * @return Temperatura en °C
     */
    static double steinhartHartTemperature(double resistance, double A, double B, double C);

    /**
     * @brief Calcula la resistencia del NTC en un puente de Wheatstone a partir del voltaje diferencial
     * @param diffVoltage Voltaje diferencial medido (V)
     * @return Resistencia medida del NTC (ohms) o -1 si hay error
     */
    static double computeNtcResistanceFromBridge(double diffVoltage);

    /**
     * @brief Calcula la resistencia del NTC en un divisor de tensión simple.
     * @param voltage Voltaje medido en el punto medio del divisor (V)
     * @param vRef Voltaje de referencia (V), típicamente 2.5V
     * @param rFixed Resistencia fija en el divisor (ohms), típicamente 10000 ohms
     * @param ntcTop Si true, el NTC está conectado a Vref; si false, está conectado a GND
     * @return Resistencia medida del NTC (ohms)
     */
    static double computeNtcResistanceFromVoltageDivider(double voltage, double vRef, double rFixed, bool ntcTop = true);

    /**
     * @brief Obtiene la temperatura de un sensor NTC100K usando ADS124S08
     * 
     * Mide diferencialmente desde el ADS124S08:
     * - Para NTC100K_0: AIN1 (+) y AIN0 (-)
     * - Para NTC100K_1: AIN3 (+) y AIN2 (-)
     * 
     * @param configKey "0" o "1" para seleccionar el sensor
     * @return Temperatura en °C o NAN en caso de error
     */
    static double readNtc100kTemperature(const char* configKey);

    /**
     * @brief Obtiene la temperatura de un sensor NTC10K usando ADS124S08
     * 
     * Mide diferencialmente entre AIN4 (+) y AINCOM (-)
     * 
     * @return Temperatura en °C o NAN en caso de error
     */
    static double readNtc10kTemperature();
};

#endif // NTC_MANAGER_H 