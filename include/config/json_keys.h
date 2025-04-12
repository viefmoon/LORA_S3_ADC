#ifndef JSON_KEYS_H
#define JSON_KEYS_H

// Namespaces
#define NAMESPACE_SYSTEM        "system"
#define NAMESPACE_SENSORS       "sensors"
#define NAMESPACE_LORAWAN       "lorawan"
#define NAMESPACE_LORA_SESSION  "lorasession"
#define NAMESPACE_SENSORS_MODBUS "sensors_modbus"
#define NAMESPACE_SENSORS_ADC    "sensors_adc"

// Claves
#define KEY_INITIALIZED         "initialized"
#define KEY_SLEEP_TIME          "sleep_time"
#define KEY_STATION_ID          "stationId"
#define KEY_DEVICE_ID           "deviceId"
#define KEY_VOLT                "volt"
#define KEY_SENSOR              "k"
#define KEY_SENSOR_ID           "id"
#define KEY_SENSOR_ID_TEMPERATURE_SENSOR "ts"
#define KEY_SENSOR_TYPE         "t"
#define KEY_SENSOR_ENABLE       "e"
#define KEY_LORA_JOIN_EUI       "joinEUI"
#define KEY_LORA_DEV_EUI        "devEUI"
#define KEY_LORA_NWK_KEY        "nwkKey"
#define KEY_LORA_APP_KEY        "appKey"
#define KEY_LORAWAN_SESSION     "lorawan_session"

// Claves para Modbus
#define KEY_MODBUS_SENSOR_ID    "id"
#define KEY_MODBUS_SENSOR_TYPE  "t"
#define KEY_MODBUS_SENSOR_ADDR  "a"
#define KEY_MODBUS_SENSOR_ENABLE "e"

// Claves para sensores ADC
#define KEY_ADC_SENSOR      "k"
#define KEY_ADC_SENSOR_ID   "id"
#define KEY_ADC_SENSOR_TYPE "t"
#define KEY_ADC_SENSOR_ENABLE "e"

// Namespaces analógicos
#define NAMESPACE_NTC100K   "ntc_100k"
#define NAMESPACE_NTC10K    "ntc_10k"
#define NAMESPACE_COND      "cond"
#define NAMESPACE_PH        "ph"

// Claves NTC100K
#define KEY_NTC100K_T1         "n100k_t1"
#define KEY_NTC100K_R1         "n100k_r1"
#define KEY_NTC100K_T2         "n100k_t2"
#define KEY_NTC100K_R2         "n100k_r2"
#define KEY_NTC100K_T3         "n100k_t3"
#define KEY_NTC100K_R3         "n100k_r3"

// Claves NTC10K
#define KEY_NTC10K_T1          "n10k_t1"
#define KEY_NTC10K_R1          "n10k_r1"
#define KEY_NTC10K_T2          "n10k_t2"
#define KEY_NTC10K_R2          "n10k_r2"
#define KEY_NTC10K_T3          "n10k_t3"
#define KEY_NTC10K_R3          "n10k_r3"

// Claves Conductividad
#define KEY_CONDUCT_CT         "c_ct"
#define KEY_CONDUCT_CC         "c_cc"
#define KEY_CONDUCT_V1         "c_v1"
#define KEY_CONDUCT_T1         "c_t1"
#define KEY_CONDUCT_V2         "c_v2"
#define KEY_CONDUCT_T2         "c_t2"
#define KEY_CONDUCT_V3         "c_v3"
#define KEY_CONDUCT_T3         "c_t3"

// Claves pH
#define KEY_PH_V1              "ph_v1"
#define KEY_PH_T1              "ph_t1"
#define KEY_PH_V2              "ph_v2"
#define KEY_PH_T2              "ph_t2"
#define KEY_PH_V3              "ph_v3"
#define KEY_PH_T3              "ph_t3"
#define KEY_PH_CT              "ph_ct"

#endif // JSON_KEYS_H 