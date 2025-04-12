#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include "config/pins_config.h"

/**
 * @brief Clase utilitaria para gestionar el control de energía de los componentes del sistema.
 * Proporciona métodos estáticos para encender/apagar las distintas líneas de alimentación.
 */
class PowerManager {
public:
    /**
     * @brief Inicializa los pines de control de energía
     */
    static void begin();

    /**
     * @brief Activa la línea de alimentación de 3.3V
     */
    static void power3V3On();
    
    /**
     * @brief Desactiva la línea de alimentación de 3.3V
     */
    static void power3V3Off();
    
    /**
     * @brief Activa la línea de alimentación de 12V1
     */
    static void power12V1On();
    
    /**
     * @brief Desactiva la línea de alimentación de 12V
     */
    static void power12V1Off();

        /**
     * @brief Activa la línea de alimentación de 12V2
     */
    static void power12V2On();
    
    /**
     * @brief Desactiva la línea de alimentación de 12V2
     */
    static void power12V2Off(); 
    
    /**
     * @brief Desactiva todas las líneas de alimentación
     */
    static void allPowerOff();
};

#endif 