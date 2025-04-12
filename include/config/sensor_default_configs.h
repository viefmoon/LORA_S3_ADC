#ifndef SENSOR_DEFAULT_CONFIGS_H
#define SENSOR_DEFAULT_CONFIGS_H

#include "sensor_types.h"

// Configuración default sensores
#define DEFAULT_SENSOR_CONFIGS { \
    {"B6", "BM6_1", BME680, true}, \
    {"C", "CO2_1", CO2, false}, \
    {"B2", "BM2_1", BME280, true}, \
    {"L", "LUX1", VEML7700, false}, \
    {"SH4", "SH4_1", SHT40, true}, \
    {"R", "RTD_1",  RTD, true}, \
    {"D", "DS_1",   DS18B20, false}, \
    {"SH3", "SH3_1", SHT30, false} \
}

// Sin sensores Modbus registrados
#define DEFAULT_MODBUS_SENSOR_CONFIGS { \
    {"ModbusEnv1", ENV4, 1, false} \
}

// Configuración default sensores ADC
#define DEFAULT_ADC_SENSOR_CONFIGS { \
    {"0", "NTC1",  N100K, true}, \
    {"1", "NTC2",  N100K, true}, \
    {"2", "NTC3",  N10K, false}, \
    {"3", "HDS10", HDS10, false}, \
    {"4", "COND",  COND, false}, \
    {"5", "SM1",   SOILH, false}, \
    {"8", "PH",    PH, false} \
}

#endif // SENSOR_DEFAULT_CONFIGS_H 