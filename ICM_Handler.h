

    //============================================
    //   3. Исправленный ICM_Handler.h
    //============================================
    //  🚀 Улучшения с буферизацией данных:
    //  5. ICM_Handler.h - Добавляем буферизацию
    //  cpp

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include "E49_Config.h"
#include "SPI_Manager.h"

class ICMHandler {
private:
    static const uint32_t READ_INTERVAL = 20;
    static const size_t BUFFER_SIZE = 100; // Буфер на 100 измерений
    
    static uint32_t last_read_time;
    static SensorData current_data;
    static uint32_t read_count;
    static uint32_t error_count;
    
    // Буфер для хранения данных перед записью на SD
    static SensorData data_buffer[BUFFER_SIZE];
    static size_t buffer_index;
    static bool buffer_overflow;

public:
    static bool begin();
    static bool readData();
    static const SensorData& getData();
    static bool isDataFresh();
    static uint32_t getReadCount();
    static uint32_t getErrorCount();
    
    // Методы для работы с буфером
    static bool addToBuffer(const SensorData& data);
    static size_t getBufferSize() { return buffer_index; }
    static bool isBufferFull() { return buffer_index >= BUFFER_SIZE; }
    static bool isBufferOverflow() { return buffer_overflow; }
    static const SensorData* getBuffer() { return data_buffer; }
    static void clearBuffer() { buffer_index = 0; buffer_overflow = false; }
    
    // Статистика буфера
    static size_t getBufferCapacity() { return BUFFER_SIZE; }
    static float getBufferUsage() { return (float)buffer_index / BUFFER_SIZE * 100.0f; }
};