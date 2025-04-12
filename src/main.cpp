#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include <ArduinoJson.h>
#include <cmath>
#include <map>
#include <string>

#include "config/pins_config.h"
#include "config/system_config.h"
#include "config/lora_config.h"
#include "debug.h"
#include "PowerManager.h"
#include <RadioLib.h>
#include <ESP32Time.h>
#include "sensor_types.h"
#include "SensorManager.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include "config_manager.h"
#include "utilities.h"
#include <SensirionI2cSht3x.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include "LoRaManager.h"
#include "BLE.h"
#include "HardwareManager.h"
#include "SleepManager.h"
#include "SHT31.h"
#include <Adafruit_MAX31865.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML7700.h>
#include <SensirionI2cSht4x.h>

//==============================================================================
// CONFIGURACIÓN INICIAL
//==============================================================================

// Bandera global para el modo de configuración
bool wokeFromConfigPin = false;

// Función de inicialización temprana
void preinit() {
  // Establecer frecuencia de CPU antes de cualquier otra inicialización
  setCpuFrequencyMhz(CPU_FREQUENCY_MHZ);
}

//==============================================================================
// VARIABLES GLOBALES Y CONFIGURACIÓN
//==============================================================================

// Gestión de energía y tiempo
ESP32Time rtc;
uint32_t timeToSleep;
unsigned long setupStartTime; // Variable para almacenar el tiempo de inicio

// Identificación del dispositivo
String deviceId;
String stationId;
bool systemInitialized;

// Almacenamiento persistente
Preferences preferences;
Preferences store;

// Configuración LoRa
const LoRaWANBand_t Region = LORA_REGION;
const uint8_t subBand = LORA_SUBBAND;

// Hardware de comunicaciones LoRa
SPIClass spiLora(FSPI);
SPISettings spiRadioSettings(SPI_LORA_CLOCK, MSBFIRST, SPI_MODE0);
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spiLora, spiRadioSettings);
LoRaWANNode node(&radio, &Region, subBand);
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

// Configuraciones de sensores habilitados
std::vector<SensorConfig> enabledNormalSensors;
std::vector<ModbusSensorConfig> enabledModbusSensors;
std::vector<SensorConfig> enabledAdcSensors;

// Objetos de sensores de temperatura y humedad
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemp(&oneWire);
SHT31 sht30Sensor(SHT31_I2C_ADDR, &Wire);
SensirionI2cSht4x sht40Sensor;

// Sensores ambientales
Adafruit_BME680 bme680Sensor(&Wire);
Adafruit_BME280 bme280Sensor;
Adafruit_VEML7700 veml7700;
SCD4x scd4x(SCD4x_SENSOR_SCD41);

// Sensores analógicos
Adafruit_MAX31865 rtdSensor = Adafruit_MAX31865(PT100_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_SCK_PIN);

// Variables para almacenar lecturas de sensores
std::vector<SensorReading> normalReadings;
std::vector<ModbusSensorReading> modbusReadings;
std::vector<SensorReading> adcReadings;

//==============================================================================
// FUNCIONES AUXILIARES
//==============================================================================

/**
 * @brief Inicializa el hardware y la configuración del sistema
 * @return true si la inicialización fue exitosa, false en caso de error
 */
bool initHardware() {
    // Liberar pines que se mantuvieron en estado específico durante el deep sleep
    SleepManager::releaseHeldPins();

    // // Inicialización del NVS y de hardware I2C/IO
    // preferences.clear();       // Comentado para evitar borrar los nonces guardados
    // nvs_flash_erase();         // Comentado para evitar borrar los nonces guardados
    // nvs_flash_init();          // Comentado para preservar datos NVS entre boots

    // Inicialización de configuración
    if (!ConfigManager::checkInitialized()) {
        ConfigManager::initializeDefaultConfig();
    }
    ConfigManager::getSystemConfig(systemInitialized, timeToSleep, deviceId, stationId);

    // Obtener configuraciones de sensores habilitados
    enabledNormalSensors = ConfigManager::getEnabledSensorConfigs();
    enabledModbusSensors = ConfigManager::getEnabledModbusSensorConfigs();
    enabledAdcSensors = ConfigManager::getEnabledAdcSensorConfigs();

    // Inicialización de hardware básico (GPIO, I2C, SPI, etc.)
    if (!HardwareManager::initHardware(spiLora, enabledNormalSensors)) {
        return false;
    }

    // Comprobar si tenemos un timestamp válido
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        // Establecer una fecha/hora predeterminada si el RTC no está configurado
        rtc.setTime(0, 0, 0, 1, 1, 2023);  // 01/01/2023 00:00:00
    }

    // Inicializar sensores
    SensorManager::beginSensors(enabledNormalSensors, enabledAdcSensors);
    DEBUG_PRINTLN("Tiempo después de iniciar sensores: " + String(millis() - setupStartTime));
    
    return true;
}

/**
 * @brief Configura y activa la radio LoRa
 * @return true si la configuración fue exitosa, false en caso de error
 */
bool configureLoRa() {
    // Inicializar radio LoRa
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        return false;
    }

    // Activar LoRaWAN
    state = LoRaManager::lwActivate(node);
    if (state != RADIOLIB_LORAWAN_NEW_SESSION && 
        state != RADIOLIB_LORAWAN_SESSION_RESTORED) {
        return false;
    }
    
    DEBUG_PRINTLN("Tiempo después de activar radio: " + String(millis() - setupStartTime));
    return true;
}

/**
 * @brief Lee los datos de todos los sensores habilitados
 */
void readSensors() {
    // Limpiar vectores de lecturas previas
    normalReadings.clear();
    modbusReadings.clear();
    adcReadings.clear();
    
    // Obtener todas las lecturas de sensores
    SensorManager::getAllSensorReadings(normalReadings, modbusReadings, adcReadings, 
                                      enabledNormalSensors, enabledModbusSensors, enabledAdcSensors);
}

/**
 * @brief Envía los datos de los sensores a través de LoRa
 */
void sendData() {
    // Apagar las fuentes de alimentación de sensores antes de enviar datos
    PowerManager::allPowerOff();
    
    // Enviar datos a través de LoRa
    LoRaManager::sendDelimitedPayload(normalReadings, modbusReadings, adcReadings, 
                                    node, deviceId, stationId, rtc);
    
    // Registrar tiempo de operación
    unsigned long elapsedTime = millis() - setupStartTime;
    DEBUG_PRINTF("Tiempo transcurrido antes de sleep: %lu ms\n", elapsedTime);
}

//==============================================================================
// SETUP
//==============================================================================
void setup() {
    // Inicializar contador de tiempo y log
    setupStartTime = millis();
    DEBUG_BEGIN(SERIAL_BAUD_RATE);
    
    // Determinar causa del despertar
    SleepManager::handleWakeupCause(wokeFromConfigPin);

    // Inicializar hardware y configuración
    if (!initHardware()) {
        SleepManager::goToDeepSleep(timeToSleep, &radio, node, LWsession, spiLora);
    }

    // Verificar y entrar en modo configuración BLE si es necesario
    if (BLEHandler::checkConfigMode()) {
        return;
    }

    // Configurar LoRa
    if (!configureLoRa()) {
        SleepManager::goToDeepSleep(timeToSleep, &radio, node, LWsession, spiLora);
    }
}

//==============================================================================
// LOOP
//==============================================================================
void loop() {
    // Verificar si se mantiene presionado para modo config
    if (BLEHandler::checkConfigMode()) {
        return;
    }

    // Leer datos de sensores
    readSensors();
    
    // Enviar datos
    sendData();
    
    // Pequeña pausa antes de dormir
    delay(10);
    
    // Entrar en modo deep sleep
    SleepManager::goToDeepSleep(timeToSleep, &radio, node, LWsession, spiLora);
}
