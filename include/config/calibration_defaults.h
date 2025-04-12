#ifndef CALIBRATION_DEFAULTS_H
#define CALIBRATION_DEFAULTS_H

// Calibración batería
#define BATTERY_R1 100000.0     // Resistencia de 100K a GND
#define BATTERY_R2 390000.0     // Resistencia de 390K a batería 
#define BATTERY_CONVERSION_FACTOR 1.0 / (BATTERY_R1 / (BATTERY_R1 + BATTERY_R2))

// Calibración NTC 100K
#define DEFAULT_T1_100K     25.0
#define DEFAULT_R1_100K     100000.0
#define DEFAULT_T2_100K     35.0
#define DEFAULT_R2_100K     64770.0
#define DEFAULT_T3_100K     45.0
#define DEFAULT_R3_100K     42530.0

// Calibración NTC 10K
#define DEFAULT_T1_10K      25.0
#define DEFAULT_R1_10K      10000.0
#define DEFAULT_T2_10K      35.0
#define DEFAULT_R2_10K      6477.0
#define DEFAULT_T3_10K      45.0
#define DEFAULT_R3_10K      4253.0

// Calibración Conductividad
#define CONDUCTIVITY_DEFAULT_V1    0.010314f
#define CONDUCTIVITY_DEFAULT_T1    84.0f
#define CONDUCTIVITY_DEFAULT_V2    0.175384f
#define CONDUCTIVITY_DEFAULT_T2    1413.0f
#define CONDUCTIVITY_DEFAULT_V3    1.407745f
#define CONDUCTIVITY_DEFAULT_T3    12880.0f
#define TEMP_COEF_COMPENSATION     0.02f
#define CONDUCTIVITY_DEFAULT_TEMP  24.22f

// Calibración pH
#define PH_DEFAULT_V1          0.4425
#define PH_DEFAULT_T1          4.01
#define PH_DEFAULT_V2          0.001
#define PH_DEFAULT_T2          6.86
#define PH_DEFAULT_V3         -0.32155
#define PH_DEFAULT_T3          9.18
#define PH_DEFAULT_TEMP        25.0

#endif // CALIBRATION_DEFAULTS_H 