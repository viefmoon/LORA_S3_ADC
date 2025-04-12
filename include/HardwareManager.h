/*******************************************************************************************
 * Archivo: include/HardwareManager.h
 * Descripción: Gestión de inicialización y configuración del hardware del sistema.
 * Incluye funciones para inicialización de periféricos y control de energía.
 *******************************************************************************************/

#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "config/system_config.h"
#include "PowerManager.h"
#include "SHT31.h"
#include "sensor_types.h"
#include <vector>
#include <Adafruit_BME680.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML7700.h>
#include <SensirionI2cSht4x.h>
#include <map>
#include <string>

class HardwareManager {
public:
    /**
     * @brief Inicializa el hardware básico del sistema (GPIO, I2C, SPI, etc.)
     * @param spiLora Referencia al bus SPI para el módulo LoRa
     * @param enabledNormalSensors Vector de sensores habilitados en la configuración
     * @return true si la inicialización fue exitosa, false en caso contrario
     */
    static bool initHardware(SPIClass& spiLora,
                           SPIClass& spiShared, // Añadir spiShared como parámetro
                           const std::vector<SensorConfig>& enabledNormalSensors);

    /**
     * @brief Inicializa los pines CS/SS para los dispositivos SPI
     */
    static void initializeSPISSPins();

};

#endif // HARDWARE_MANAGER_H
