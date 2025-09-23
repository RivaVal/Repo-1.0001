


//====================================================================
//	2. Файл SPI_Manager.h
//====================================================================
//  Как и требовалось, нужно добавить тесты.
//  Улучшенная реализация
//  1. Исправленный SPI_Manager.h
//
#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "Config.h"

class SPIManager {
private:
    static bool spi_busy;
    static uint32_t last_operation_time;
    static uint8_t current_cs_pin;

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

