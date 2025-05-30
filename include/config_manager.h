#pragma once
#include <Preferences.h>
#include <vector>
#include <Arduino.h>  // Se incluye para utilizar el tipo String
#include "sensor_types.h"
#include <RadioLib.h> // Añadido para RADIOLIB_LORAWAN_SESSION_BUF_SIZE

// Incluimos los archivos de configuración necesarios
#include "config/system_config.h"
#include "config/json_keys.h"
#include "config/lora_config.h"
#include "config/sensor_default_configs.h"
#include "config/calibration_defaults.h"
#include "config/sensor_defaults.h"
#include "config/pins_config.h"

// Definición de la estructura para la configuración de LoRa
struct LoRaConfig {
    //FOR OTAA
    String joinEUI;
    String devEUI;
    String nwkKey;
    String appKey;
};

class ConfigManager {
public:
    /* =========================================================================
       INICIALIZACIÓN Y CONFIGURACIÓN DEL SISTEMA
       ========================================================================= */
    // Verificación e inicialización
    static bool checkInitialized();
    static void initializeDefaultConfig();
    
    // Configuración del sistema
    static void getSystemConfig(bool &initialized, uint32_t &sleepTime, String &deviceId, String &stationId);
    static void setSystemConfig(bool initialized, uint32_t sleepTime, const String &deviceId, const String &stationId);

    /* =========================================================================
       CONFIGURACIÓN DE SENSORES NO-MODBUS
       ========================================================================= */
    // Gestión de sensores generales
    static void setSensorsConfigs(const std::vector<SensorConfig>& configs);
    static std::vector<SensorConfig> getAllSensorConfigs();
    static std::vector<SensorConfig> getEnabledSensorConfigs();

    /* =========================================================================
       CONFIGURACIÓN DE SENSORES MODBUS
       ========================================================================= */
    static void setModbusSensorsConfigs(const std::vector<ModbusSensorConfig>& configs);
    static std::vector<ModbusSensorConfig> getAllModbusSensorConfigs();
    static std::vector<ModbusSensorConfig> getEnabledModbusSensorConfigs();
    
    /* =========================================================================
       CONFIGURACIÓN DE SENSORES ADC
       ========================================================================= */
    static void setAdcSensorsConfigs(const std::vector<SensorConfig>& configs);
    static std::vector<SensorConfig> getAllAdcSensorConfigs();
    static std::vector<SensorConfig> getEnabledAdcSensorConfigs();
    
    /* =========================================================================
       CONFIGURACIÓN DE LORA
       ========================================================================= */
    static LoRaConfig getLoRaConfig();
    static void setLoRaConfig(
        const String &joinEUI, 
        const String &devEUI, 
        const String &nwkKey, 
        const String &appKey);
    
    /* =========================================================================
       CONFIGURACIÓN DE SENSORES ANALÓGICOS
       ========================================================================= */
    // NTC 100K
    static void getNTC100KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3);
    static void setNTC100KConfig(double t1, double r1, double t2, double r2, double t3, double r3);
    
    // NTC 10K
    static void getNTC10KConfig(double& t1, double& r1, double& t2, double& r2, double& t3, double& r3);
    static void setNTC10KConfig(double t1, double r1, double t2, double r2, double t3, double r3);
    
    // Conductividad
    static void getConductivityConfig(float& calTemp, float& coefComp, 
                                    float& v1, float& t1, float& v2, float& t2, float& v3, float& t3);
    static void setConductivityConfig(float calTemp, float coefComp,
                                    float v1, float t1, float v2, float t2, float v3, float t3);
    
    // pH
    static void getPHConfig(float& v1, float& t1, float& v2, float& t2, float& v3, float& t3, float& defaultTemp);
    static void setPHConfig(float v1, float t1, float v2, float t2, float v3, float t3, float defaultTemp);

private:
    // Configuraciones por defecto
    static const SensorConfig defaultConfigs[]; // Configs no-Modbus
    static const ModbusSensorConfig defaultModbusSensors[]; // Configs Modbus
    static const SensorConfig defaultAdcSensors[]; // Configs ADC
};

