#include "SensorManager.h"
#include <Wire.h>
#include <SPI.h>
#include <cmath>  // Para fabs() y otras funciones matemáticas
#include <DallasTemperature.h>
#include "sensor_types.h"
#include "config/pins_config.h"
#include "config/sensor_defaults.h"
#include <Preferences.h>
#include "config_manager.h"
#include "debug.h"
#include "utilities.h"
#include <map>
#include <string>

// Eliminadas las inclusiones de "ADS124S08.h" y "AdcUtilities.h"
#include "sensors/NtcManager.h"
#include "sensors/PHSensor.h"
#include "sensors/ConductivitySensor.h"
#include "sensors/HDS10Sensor.h"
// Se eliminó la variable externa ADC

// Inclusión de sensores
#include "sensors/SHT30Sensor.h"
#include "sensors/DS18B20Sensor.h"
#include "sensors/CO2Sensor.h" // Añadir inclusión para sensor CO2
#include "sensors/VEML7700Sensor.h" // Añadir inclusión para sensor VEML7700
#include "sensors/SHT40Sensor.h" // Añadir inclusión para sensor SHT40
#include <Adafruit_BME680.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML7700.h>

// Variables externas
extern Adafruit_BME680 bme680Sensor;
extern Adafruit_BME280 bme280Sensor;
extern Adafruit_VEML7700 veml7700;

// Inicialización del mapa estático
std::map<std::string, bool> SensorManager::sensorInitStatus;

// -------------------------------------------------------------------------------------
// Métodos de la clase SensorManager
// -------------------------------------------------------------------------------------

// Implementación de los nuevos métodos para gestionar el estado de inicialización
bool SensorManager::isSensorInitialized(const std::string& sensorId) {
    return sensorInitStatus.count(sensorId) > 0 && sensorInitStatus[sensorId];
}

void SensorManager::setSensorInitialized(const std::string& sensorId, bool initialized) {
    sensorInitStatus[sensorId] = initialized;
}

void SensorManager::beginSensors(const std::vector<SensorConfig>& enabledNormalSensors,
                              const std::vector<SensorConfig>& enabledAdcSensors) {
    // Encender alimentación 3.3V
    PowerManager::power3V3On();
    
    // Inicializar sensores I2C habilitados
    for (const auto& sensor : enabledNormalSensors) {
        if (!sensor.enable) continue; // Saltar sensores deshabilitados

        // Obtener el identificador del sensor para mapear su estado
        std::string currentSensorId = sensor.sensorId;
        bool success = false;

        switch (sensor.type) {
            case SHT30:
                success = sht30Sensor.begin();
                if (success) sht30Sensor.reset();
                break;
            case SHT40:
                success = SHT40Sensor::begin();
                break;
            case CO2:
                success = CO2Sensor::begin();
                break;
            case BME680:
                success = bme680Sensor.begin();
                if (success) {
                    bme680Sensor.setTemperatureOversampling(BME680_OS_8X);
                    bme680Sensor.setHumidityOversampling(BME680_OS_2X);
                    bme680Sensor.setPressureOversampling(BME680_OS_4X);
                    bme680Sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
                    bme680Sensor.setGasHeater(320, 150); // 320°C durante 150 ms
                }
                break;
            case BME280:
                success = bme280Sensor.begin(BME280_I2C_ADDR, &Wire);
                if (success) {
                    bme280Sensor.setSampling(Adafruit_BME280::MODE_FORCED,
                                          Adafruit_BME280::SAMPLING_X1, // temperature
                                          Adafruit_BME280::SAMPLING_X1, // pressure
                                          Adafruit_BME280::SAMPLING_X1, // humidity
                                          Adafruit_BME280::FILTER_OFF);
                }
                break;
            case VEML7700:
                success = VEML7700Sensor::begin();
                break;
            default:
                // Para sensores no-I2C, se manejará en las secciones correspondientes abajo
                continue;
        }
        
        // Actualizar el estado en el mapa usando el nuevo método
        setSensorInitialized(currentSensorId, success);
    }
    
    // Verificar si hay algún sensor RTD
    bool rtdSensorEnabled = false;
    for (const auto& sensor : enabledNormalSensors) {
        if (sensor.type == RTD && sensor.enable) {
            rtdSensorEnabled = true;
            std::string currentSensorId = sensor.sensorId;
            break;
        }
    }
    
    // Inicializar RTD solo si está habilitado en la configuración
    if (rtdSensorEnabled) {
        rtdSensor.begin(MAX31865_4WIRE);
        // Registrar el estado de inicialización del RTD
        for (const auto& sensor : enabledNormalSensors) {
            if (sensor.type == RTD && sensor.enable) {
                std::string currentSensorId = sensor.sensorId;
                setSensorInitialized(currentSensorId, true);
            }
        }
    }

    // Verificar si hay algún sensor DS18B20
    bool ds18b20SensorEnabled = false;
    for (const auto& sensor : enabledNormalSensors) {
        if (sensor.type == DS18B20 && sensor.enable) {
            ds18b20SensorEnabled = true;
            break;
        }
    }

    // Inicializar DS18B20 solo si está habilitado en la configuración
    if (ds18b20SensorEnabled) {
        // TIEMPO ejecución ≈ 65 ms
        // Inicializar DS18B20
        dallasTemp.begin();
        dallasTemp.requestTemperatures();
        
        // Registrar el estado de inicialización del DS18B20
        for (const auto& sensor : enabledNormalSensors) {
            if (sensor.type == DS18B20 && sensor.enable) {
                std::string currentSensorId = sensor.sensorId;
                setSensorInitialized(currentSensorId, true);
            }
        }
    }

    // Configurar los pines analógicos para cada sensor
    // NTC100K 0 - Pin A0
    pinMode(NTC100K_0_PIN, INPUT);
    // NTC100K 1 - Pin A3
    pinMode(NTC100K_1_PIN, INPUT);
    // NTC10K - Pin A4
    pinMode(NTC10K_PIN, INPUT);
    // PH Sensor - Pin A5
    pinMode(PH_SENSOR_PIN, INPUT);
    // Conductivity Sensor - Pin A6
    pinMode(COND_SENSOR_PIN, INPUT);
    // HDS10 Sensor - Pin A7
    pinMode(HDS10_SENSOR_PIN, INPUT);
    // Battery Sensor - Pin A8
    pinMode(BATTERY_SENSOR_PIN, INPUT);
    // Soil Humidity Sensor
    pinMode(SOILH_SENSOR_PIN, INPUT);

    // Inicializar sensores ADC habilitados
    for (const auto& sensor : enabledAdcSensors) {
        if (!sensor.enable) continue; // Saltar sensores deshabilitados

        // Obtener el identificador del sensor para mapear su estado
        std::string currentSensorId = sensor.sensorId;
        bool success = true; // Por defecto éxito para sensores ADC
        
        // Registrar en el mapa de estado usando el nuevo método
        setSensorInitialized(currentSensorId, success);
    }

    // Configurar ADC interno
    analogReadResolution(13); // Resolución de 12 bits (0-4095)
    analogSetAttenuation(ADC_11db); // Atenuación para medir hasta 3.3V
}

SensorReading SensorManager::getSensorReading(const SensorConfig &cfg) {
    SensorReading reading;
    strncpy(reading.sensorId, cfg.sensorId, sizeof(reading.sensorId) - 1);
    reading.sensorId[sizeof(reading.sensorId) - 1] = '\0';
    reading.type = cfg.type;
    reading.value = NAN;

    readSensorValue(cfg, reading);

    return reading;
}

/**
 * @brief Lógica principal para leer el valor de cada sensor normal (no Modbus) según su tipo.
 */
float SensorManager::readSensorValue(const SensorConfig &cfg, SensorReading &reading) {
    // Verificar si el sensor está inicializado usando el nuevo método
    std::string currentSensorId = cfg.sensorId;
    
    if (!isSensorInitialized(currentSensorId)) {
        // Si el sensor no está inicializado

        reading.value = NAN; // Asignar NAN al valor principal
        reading.subValues.clear(); // Limpiar subvalores

        // Para sensores multivalor, llenar subValues con NAN para mantener la estructura esperada
        size_t numSubValues = 0;
        switch(cfg.type) {
            case SHT30: numSubValues = 2; break;
            case CO2:
            case BME680:
            case BME280: numSubValues = 3; break;
            // El resto de sensores son de valor único
        }

        if (numSubValues > 0) {
            SubValue nanVal; nanVal.value = NAN;
            for(size_t i = 0; i < numSubValues; ++i) {
                reading.subValues.push_back(nanVal);
            }
        }

        return NAN; // Retornar NAN inmediatamente
    }

    // Si la inicialización fue exitosa, proceder con la lectura normal
    switch (cfg.type) {
        case N100K:
            // Usar NtcManager para obtener la temperatura
            reading.value = NtcManager::readNtc100kTemperature(cfg.configKey);
            break;

        case N10K:
            // Usar NtcManager para obtener la temperatura del NTC de 10k
            reading.value = NtcManager::readNtc10kTemperature();
            break;
            
        case HDS10:
            // Leer sensor HDS10 y obtener el porcentaje de humedad
            reading.value = HDS10Sensor::read();
            break;
            
        case PH:
            // Leer sensor de pH y obtener valor
            reading.value = PHSensor::read();
            break;

        case COND:
            // Leer sensor de conductividad y obtener valor en ppm
            reading.value = ConductivitySensor::read();
            break;
        
        case SOILH:
            {
                // Leer el valor del pin analógico
                int adcValue = analogRead(SOILH_SENSOR_PIN);
                
                // Convertir el valor ADC a voltaje (0-3.3V con resolución de 12 bits)
                float voltage = adcValue * (3.3f / 4095.0f);
                
                // Verificar si el voltaje está en rango válido
                if (voltage <= 0.0f || voltage >= 3.3f) {
                    reading.value = NAN;
                } else {
                    // Convertir el voltaje a porcentaje (0V = 0%, 3.3V = 100%)
                    reading.value = (voltage / 3.3f) * 100.0f;
                }
                DEBUG_PRINTF("SOILH ADC: %d, voltaje: %.3f, valor: %.3f%%\n", adcValue, voltage, reading.value);
            }
            break;

        case RTD:
            {
                uint16_t rtdValue = rtdSensor.readRTD();
                float temp = rtdSensor.temperature(RNOMINAL, RREF);
                
                // Verificar si hay errores
                uint8_t fault = rtdSensor.readFault();
                if (fault) {
                    rtdSensor.clearFault();
                    reading.value = NAN;
                } else {
                    reading.value = temp;
                }
            }
            break;

        case DS18B20:
            reading.value = DS18B20Sensor::read();
            break;

        case SHT30:
        {
            float tmp = 0.0f, hum = 0.0f;
            SHT30Sensor::read(tmp, hum);
            reading.subValues.clear();
            
            // Agregar temperatura como primer valor [0]
            {
                SubValue sT; 
                sT.value = tmp;
                reading.subValues.push_back(sT);
            }
            
            // Agregar humedad como segundo valor [1]
            {
                SubValue sH; 
                sH.value = hum;
                reading.subValues.push_back(sH);
            }
            
            // Asignar el valor principal como NAN si alguno de los valores falló
            reading.value = (isnan(tmp) || isnan(hum)) ? NAN : tmp;
        }
        break;

        case SHT40:
        {
            float tmp = 0.0f, hum = 0.0f;
            bool success = SHT40Sensor::read(tmp, hum);
            reading.subValues.clear();
            
            if (success) {
                // Agregar temperatura como primer valor [0]
                SubValue sT; 
                sT.value = tmp;
                reading.subValues.push_back(sT);
                
                // Agregar humedad como segundo valor [1]
                SubValue sH; 
                sH.value = hum;
                reading.subValues.push_back(sH);
                
                // Asignar temperatura como valor principal
                reading.value = tmp;
            } else {
                // En caso de error, rellenar con NAN
                SubValue nanVal; nanVal.value = NAN;
                reading.subValues.push_back(nanVal); // Temperatura
                reading.subValues.push_back(nanVal); // Humedad
                reading.value = NAN;
            }
        }
        break;

        // Caso para el sensor de CO2 (SCD4x)
        case CO2:
        {
            float co2 = NAN, temp = NAN, hum = NAN; // Inicializar a NAN
            if (CO2Sensor::read(co2, temp, hum)) {
                // Lectura exitosa
                reading.subValues.clear();

                // Agregar CO2 como primer valor [0]
                SubValue sCO2;
                sCO2.value = co2;
                reading.subValues.push_back(sCO2);

                // Agregar Temperatura como segundo valor [1]
                SubValue sTemp;
                sTemp.value = temp;
                reading.subValues.push_back(sTemp);

                // Agregar Humedad como tercer valor [2]
                SubValue sHum;
                sHum.value = hum;
                reading.subValues.push_back(sHum);

                // Asignar CO2 como valor principal
                reading.value = co2;
            } else {
                // Lectura fallida, asegurar que todo sea NAN
                reading.subValues.clear();
                SubValue nanVal; nanVal.value = NAN;
                reading.subValues.push_back(nanVal); // CO2
                reading.subValues.push_back(nanVal); // Temp
                reading.subValues.push_back(nanVal); // Hum
                reading.value = NAN;
            }
        }
        break;
        
        case BME680:
        {
            // performReading() puede tardar un poco
            if (!bme680Sensor.performReading()) {
                reading.value = NAN;
                reading.subValues.clear();
                // Rellenar con NANs para mantener la estructura
                SubValue nanVal; nanVal.value = NAN;
                reading.subValues.push_back(nanVal); // Temp
                reading.subValues.push_back(nanVal); // Hum
                reading.subValues.push_back(nanVal); // Press
                reading.subValues.push_back(nanVal); // Gas
            } else {
                reading.subValues.clear();
                float temp = bme680Sensor.temperature;
                float hum = bme680Sensor.humidity;
                float press = bme680Sensor.pressure / 100.0F; // Convertir Pa a hPa
                float gas = bme680Sensor.gas_resistance / 1000.0F; // Convertir Ohms a KOhms

                // Rellenar subValues en orden definido: [0]=Temp, [1]=Hum, [2]=Press, [3]=Gas
                SubValue svT; svT.value = temp; reading.subValues.push_back(svT);
                SubValue svH; svH.value = hum; reading.subValues.push_back(svH);
                SubValue svP; svP.value = press; reading.subValues.push_back(svP);
                SubValue svG; svG.value = gas; reading.subValues.push_back(svG);

                // Asignar un valor principal (ej. temperatura)
                reading.value = temp;
            }
        }
        break;

        case BME280:
        {
            // Forzar una medición (necesario en MODE_FORCED)
            bme280Sensor.takeForcedMeasurement();

            // Leer valores
            float temp = bme280Sensor.readTemperature();    // Lee Temperatura en °C
            float hum = bme280Sensor.readHumidity();       // Lee Humedad en %
            float press = bme280Sensor.readPressure();     // Lee Presión en Pa

            // Verificar si las lecturas son válidas (la librería suele devolver NAN en error)
            if (isnan(temp) || isnan(hum) || isnan(press)) {
                reading.value = NAN;
                reading.subValues.clear();
                // Rellenar con NANs para mantener la estructura
                SubValue nanVal; nanVal.value = NAN;
                reading.subValues.push_back(nanVal); // Temp
                reading.subValues.push_back(nanVal); // Hum
                reading.subValues.push_back(nanVal); // Press
            } else {
                // Convertir presión de Pa a hPa
                press = press / 100.0F;

                reading.subValues.clear();

                // Rellenar subValues en orden definido: [0]=Temp(°C), [1]=Hum(%), [2]=Press(hPa)
                SubValue svT; svT.value = temp; reading.subValues.push_back(svT);
                SubValue svH; svH.value = hum; reading.subValues.push_back(svH);
                SubValue svP; svP.value = press; reading.subValues.push_back(svP);

                // Asignar un valor principal (ej. temperatura)
                reading.value = temp;
            }
        }
        break;

        case VEML7700:
        {
            // Leer el valor de luz en lux
            float lux = VEML7700Sensor::read();
            
            // Verificar si la lectura es válida
            if (isnan(lux)) {
                reading.value = NAN;
                reading.subValues.clear();
                // Rellenar con NANs para mantener la estructura
                SubValue nanVal; nanVal.value = NAN;
                reading.subValues.push_back(nanVal); // Lux
            } else {
                reading.subValues.clear();
                
                // Rellenar subValues: [0]=Lux
                SubValue svL; svL.value = lux; reading.subValues.push_back(svL);
                
                // Asignar el valor de lux como valor principal
                reading.value = lux;
            }
        }
        break;

        default:
            reading.value = NAN;
            break;
    }
    return reading.value;
}

ModbusSensorReading SensorManager::getModbusSensorReading(const ModbusSensorConfig& cfg) {
    ModbusSensorReading reading;
    
    // Copiar el ID del sensor
    strlcpy(reading.sensorId, cfg.sensorId, sizeof(reading.sensorId));
    reading.type = cfg.type;
    
    // Leer sensor según su tipo
    switch (cfg.type) {
        case ENV4:
            reading = ModbusSensorManager::readEnvSensor(cfg);
            break;
        // Añadir casos para otros tipos de sensores Modbus
        default:
            DEBUG_PRINTLN("Tipo de sensor Modbus no soportado");
            break;
    }
    
    return reading;
}

void SensorManager::getAllSensorReadings(std::vector<SensorReading>& normalReadings,
                                        std::vector<ModbusSensorReading>& modbusReadings,
                                        std::vector<SensorReading>& adcReadings,
                                        const std::vector<SensorConfig>& enabledNormalSensors,
                                        const std::vector<ModbusSensorConfig>& enabledModbusSensors,
                                        const std::vector<SensorConfig>& enabledAdcSensors) {
    // Reservar espacio para los vectores
    normalReadings.reserve(enabledNormalSensors.size());
    modbusReadings.reserve(enabledModbusSensors.size());
    adcReadings.reserve(enabledAdcSensors.size());
    
    // Leer sensores normales
    for (const auto &sensor : enabledNormalSensors) {
        normalReadings.push_back(getSensorReading(sensor));
    }
    
    // Leer sensores ADC
    for (const auto &sensor : enabledAdcSensors) {
        adcReadings.push_back(getSensorReading(sensor));
    }
    
    // Si hay sensores Modbus, inicializar comunicación, leerlos y finalizar
    if (!enabledModbusSensors.empty()) {
        // Determinar el tiempo máximo de estabilización necesario
        uint32_t maxStabilizationTime = 0;
        
        // Revisar cada sensor habilitado para encontrar el tiempo máximo
        for (const auto &sensor : enabledModbusSensors) {
            uint32_t sensorStabilizationTime = 0;
            
            // Obtener el tiempo de estabilización según el tipo de sensor
            switch (sensor.type) {
                case ENV4:
                    sensorStabilizationTime = MODBUS_ENV4_STABILIZATION_TIME;
                    break;
                // Añadir casos para otros tipos de sensores Modbus con sus respectivos tiempos
                default:
                    sensorStabilizationTime = 500; // Tiempo predeterminado si no se especifica
                    break;
            }
            
            // Actualizar el tiempo máximo si este sensor necesita más tiempo
            if (sensorStabilizationTime > maxStabilizationTime) {
                maxStabilizationTime = sensorStabilizationTime;
            }
        }
        
        // Encender alimentación de 12V para sensores Modbus
        PowerManager::power12VOn();
        delay(maxStabilizationTime);
        
        // Inicializar comunicación Modbus antes de comenzar las mediciones
        ModbusSensorManager::beginModbus();
        
        // Leer todos los sensores Modbus
        for (const auto &sensor : enabledModbusSensors) {
            modbusReadings.push_back(getModbusSensorReading(sensor));
        }
        
        // Finalizar comunicación Modbus después de completar todas las lecturas
        ModbusSensorManager::endModbus();
        
        // Apagar alimentación de 12V después de completar las lecturas
        PowerManager::power12VOff();
    }
}
