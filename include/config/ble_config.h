#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

// UUIDs del servicio BLE
#define BLE_SERVICE_UUID             "180A"
#define BLE_CHAR_SYSTEM_UUID         "2A37"
#define BLE_CHAR_SENSORS_UUID        "2A40"
#define BLE_CHAR_LORA_CONFIG_UUID    "2A41"
#define BLE_CHAR_NTC100K_UUID        "2A38"
#define BLE_CHAR_NTC10K_UUID         "2A39"
#define BLE_CHAR_CONDUCTIVITY_UUID   "2A3C"
#define BLE_CHAR_PH_UUID             "2A3B"
#define BLE_DEVICE_PREFIX            "AGRICOS-"

// Configuración de tiempos BLE
#define CONFIG_TRIGGER_TIME 5000
#define CONFIG_TIMEOUT      30000
#define CONFIG_BLE_WAIT_TIMEOUT     60000   // Tiempo máximo de espera para conexión BLE (60 segundos)
#define CONFIG_BLE_MAX_CONN_TIME    300000  // Tiempo máximo de conexión BLE activa (5 minutos)

#endif // BLE_CONFIG_H 