


    //=======================================================
    //  3. МОДУЛЬ №3: SD Card Handler (SD_Handler.h)
    //=======================================================
    //  cpp
    //
    //  🚀 Улучшения с буферизацией и проверкой места:
    //  5. SD_Handler.h - Полная версия с буфером
    //cpp

#pragma once
#include "SPI_Manager.h"
#include "Config.h"
#include <SD.h>

class SDHandler {
private:
    static const uint32_t WRITE_INTERVAL = 500;
    static const size_t BUFFER_SIZE = 2048; // 2KB буфер
    static const uint32_t MIN_FREE_SPACE = 1024 * 1024; // 1MB минимально свободного места
    
    static uint32_t last_write_time;
    static uint32_t last_flush_time;
    static File data_file;
    static bool sd_initialized;
    
    // Буферизация данных
    static uint8_t write_buffer[BUFFER_SIZE];
    static size_t buffer_index;
    static uint32_t write_count;
    static uint32_t error_count;

public:
    static bool begin();
    static bool writeData(const SensorData& data);
    static void end();
    static bool isInitialized() { return sd_initialized; }
    
    // Буферизация
    static bool flushBuffer();
    
    // Проверка свободного места
    static bool checkFreeSpace();
    static uint64_t getFreeSpace();
    
    // Статистика
    static uint32_t getWriteCount() { return write_count; }
    static uint32_t getErrorCount() { return error_count; }
    static float getSuccessRate();
};