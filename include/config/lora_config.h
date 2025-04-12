#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

// Configuración de reloj SPI para LoRa
#define SPI_LORA_CLOCK       1000000

// Tamaño máximo de payload LoRa
#define MAX_LORA_PAYLOAD     200

// LoRa (OTAA) - Valores por defecto
#define DEFAULT_JOIN_EUI    "00,00,00,00,00,00,00,00"
#define DEFAULT_DEV_EUI     "1f,d4,e6,68,46,8c,e1,b7"
#define DEFAULT_APP_KEY     "1d,fb,69,80,69,d6,a0,7e,5d,bf,29,ba,6b,37,d3,04"
#define DEFAULT_NWK_KEY     "82,91,e9,55,19,ab,c0,6c,86,25,63,68,e7,f4,5a,89"

// LoRa Region y SubBand
#define LORA_REGION         US915
#define LORA_SUBBAND        2       // For US915, use 2; for other regions, use 0

#endif // LORA_CONFIG_H 