
    //
    //   3. Исправленный ICM_Handler.h
    //cpp

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include "E49_Config.h"
#include "SPI_Manager.h"

class ICMHandler {
private:
    static const uint32_t READ_INTERVAL = 20;
    static uint32_t last_read_time;
    static SensorData current_data;
    static uint32_t read_count;
    static uint32_t error_count;

public:
    static bool begin();
    static bool readData();
    static const SensorData& getData();
    static bool isDataFresh();
    static uint32_t getReadCount();
    static uint32_t getErrorCount();
};
