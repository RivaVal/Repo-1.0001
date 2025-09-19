



//====================================================================
//	2. Файл E49_Controller.h
//=================================================================

//#ifndef E49_CONTROLLER_H
//#define E49_CONTROLLER_H

// src/radio/e49_controller.h
#pragma once


#include <Arduino.h>
                            //#include <stdbool.h>
                            //#include <stdint.h>
                            //#include <stddef.h>
#include <HardwareSerial.h>
#include <EBYTE.h>
#include "E49_Config.h"


//----------

class E49_Controller {
private:
    uint8_t m0_pin;
    uint8_t m1_pin;
    uint8_t aux_pin;
    bool is_initialized;
    uint32_t operation_timeout;
    uint8_t calculateCRC8_op(const uint8_t* data, size_t length) const;
    
public:
    E49_Controller();
    
    // Совместимость со старым кодом
    bool init(uint8_t m0, uint8_t m1, uint8_t aux);
    bool sendData(const uint8_t* data, size_t length);
    bool waitForAux(uint32_t timeout_ms = 1000);
    
    // Новые методы
    bool isReady();
    bool softReset();
    bool recover();
    void setTimeout(uint32_t timeout_ms);
    bool initialized() const;
};

//====================================================================
