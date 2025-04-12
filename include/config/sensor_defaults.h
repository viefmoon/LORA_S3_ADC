#ifndef SENSOR_DEFAULTS_H
#define SENSOR_DEFAULTS_H

// Direcciones I2C para sensores
#define BME280_I2C_ADDR 0x76
#define SHT31_I2C_ADDR  0x44   // Dirección I2C para SHT31

// Configuración MAX31865 para RTD (PT100/PT1000)
#define RREF      430.0         // Resistencia de referencia en ohms
#define RNOMINAL  100.0         // Resistencia nominal del sensor a 0°C (100.0 para PT100, 1000.0 para PT1000)

// Configuración SPI RTD
#define SPI_RTD_CLOCK        1000000

// Límites de temperatura NTC para evitar lecturas erróneas cuando esta desconectado
#define NTC_TEMP_MIN           -20.0   // Temperatura mínima válida en °C
#define NTC_TEMP_MAX            100.0   // Temperatura máxima válida en °C

// Tiempos de estabilización
#define POWER_STABILIZE_DELAY   1

#endif // SENSOR_DEFAULTS_H 