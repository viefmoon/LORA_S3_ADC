#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// Pines one wire
#define ONE_WIRE_BUS        33

// Pines I2C
#define I2C_SDA_PIN         48
#define I2C_SCL_PIN         47

//Serial 1
#define SERIAL1_RX_PIN      44
#define SERIAL1_TX_PIN      43

// Pines para activar medición de batería
#define BATTERY_CONTROL_PIN 37

// Pines analógicos para sensores
#define BATTERY_SENSOR_PIN  1  // IO1 

// SPI PARA LORA
#define SPI_LORA_SCK_PIN        9
#define SPI_LORA_MISO_PIN       11
#define SPI_LORA_MOSI_PIN       10
#define LORA_NSS_PIN            8
#define LORA_BUSY_PIN           13
#define LORA_RST_PIN            12
#define LORA_DIO1_PIN           14

// SPI PARA RTD
#define SPI_SCK_PIN        39
#define SPI_MISO_PIN       40
#define SPI_MOSI_PIN       41

// PT100
#define PT100_CS_PIN        34

// Modo Config
#define CONFIG_PIN          3
#define CONFIG_LED_PIN      35

//Leds
#define LED1_PIN      21
#define LED2_PIN      19

// FlowSensor
#define FLOW_SENSOR_PIN         4

// Batería
#define POWER_3V3_PIN           36
#define POWER_12V1_PIN           6
#define POWER_12V2_PIN           7

// Modbus
#define MODBUS_RX_PIN           18
#define MODBUS_TX_PIN           20

//ADS124S08
#define ADS124S08_CS_PIN        38
#define ADS124S08_DRDY_PIN      35
#define ADS124S08_START_PIN     42
#define ADS124S08_RESET_PIN     26
#define ADS124S08_CKEN_PIN      -1

#endif // PINS_CONFIG_H 