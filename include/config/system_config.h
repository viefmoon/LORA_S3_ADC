#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// Configuración de la CPU
#define CPU_FREQUENCY_MHZ      40  // Frecuencia del CPU en MHz (40MHz o 80MHz funcionan bien en bajo consumo)

// Configuración de depuración - Comentar para deshabilitar mensajes de depuración
#define DEBUG_ENABLED

// Serial
#define SERIAL_BAUD_RATE         115200

// Deep Sleep
#define DEFAULT_TIME_TO_SLEEP   30

// Identificadores
#define DEFAULT_DEVICE_ID   "DEV01"
#define DEFAULT_STATION_ID  "ST001"

// Tamaños de documentos JSON - Centralizados
#define JSON_DOC_SIZE_SMALL   300
#define JSON_DOC_SIZE_MEDIUM  1024
#define JSON_DOC_SIZE_LARGE   2048

// Configuración Modbus
#define MODBUS_BAUD_RATE        9600
#define MODBUS_SERIAL_CONFIG    SERIAL_8N1
#define MODBUS_RESPONSE_TIMEOUT 300  // Tiempo de espera para respuesta en ms
#define MODBUS_MAX_RETRY        3     // Número máximo de intentos de lectura Modbus

#endif // SYSTEM_CONFIG_H 