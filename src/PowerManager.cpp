#include "PowerManager.h"
#include "config/sensor_defaults.h" // Para POWER_STABILIZE_DELAY

// Definir constante localmente (no redefinir si ya existe)
#ifndef POWER_STABILIZE_DELAY
#define POWER_STABILIZE_DELAY 1
#endif

void PowerManager::begin() {
    // Configurar pines como salidas
    pinMode(POWER_3V3_PIN, OUTPUT);
    pinMode(POWER_12V1_PIN, OUTPUT);
    pinMode(POWER_12V2_PIN, OUTPUT);
    // Asegurar que todas las fuentes están apagadas al inicio
    allPowerOff();
}

void PowerManager::power3V3On() {
    digitalWrite(POWER_3V3_PIN, LOW);
    delay(POWER_STABILIZE_DELAY);
}

void PowerManager::power3V3Off() {
    digitalWrite(POWER_3V3_PIN, HIGH);
}

void PowerManager::power12V1On() {
    digitalWrite(POWER_12V1_PIN, HIGH);
    delay(POWER_STABILIZE_DELAY);
}

void PowerManager::power12V1Off() {
    digitalWrite(POWER_12V1_PIN, LOW);
}

void PowerManager::power12V2On() {
    digitalWrite(POWER_12V2_PIN, HIGH);
    delay(POWER_STABILIZE_DELAY);
}

void PowerManager::power12V2Off() {
    digitalWrite(POWER_12V2_PIN, LOW);
}

void PowerManager::allPowerOff() {
    power3V3Off();
    power12V1Off();
    power12V2Off();
}