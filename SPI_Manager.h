

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <cstdint>          // Добавить для uint32_t
#include "E49_Config.h"     // Добавить для SensorData
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <cstdint>

// Временно отключим конфликтующие макросы
#ifdef ICM_CS
#undef ICM_CS
#endif

#ifdef SD_CS  
#undef SD_CS
#endif

// Используем новые имена из E49_Config.h
static const uint8_t SPI_ICM_CS = E49_ICM_CS;  // Используем макросы
static const uint8_t SPI_SD_CS = E49_SD_CS;    // Используем макросы

class SPIManager {
private:
    // Переименуем переменные чтобы избежать конфликтов
    static const uint8_t SPI_SCK  = 18;
    static const uint8_t SPI_MISO = 19;
    static const uint8_t SPI_MOSI = 23;
    static const uint8_t SPI_ICM_CS = 22;  // Переименовано!
    static const uint8_t SPI_SD_CS = 33;   // Переименовано!

    static bool spi_busy;
    static uint32_t last_operation_time;

public:
    static bool begin();
    static bool acquireForICM(uint32_t timeout = 5);
    static bool acquireForSD(uint32_t timeout = 50);
    static void release();
    static SPIClass& getSPI() { return SPI; }
    static bool isBusy() { return spi_busy; }
    static uint32_t getLastOpTime() { return last_operation_time; }

private:
    static bool acquireSPI(uint8_t cs_pin, uint32_t timeout);
};