

    //
    //  3. МОДУЛЬ №3: SD Card Handler (SD_Handler.h)
    //  cpp
    //

#pragma once
#include "SPI_Manager.h"
#include "E49_Config.h"
#include <SD.h>

class SDHandler {
private:
    static const uint32_t WRITE_INTERVAL = 500; // 500ms = 2Hz
    static uint32_t last_write_time;
    static File data_file;
    static bool sd_initialized;

public:
    static bool begin() {
        if (!SPIManager::acquireForSD(100)) {
            return false;
        }

        // Инициализация SD карты
        SPIManager::getSPI().beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE0));
        sd_initialized = SD.begin(E49_SD_CS); // Вместо SD_CS
        
        if (sd_initialized) {
            data_file = SD.open("/data.bin", FILE_WRITE);
        }
        
        SPIManager::getSPI().endTransaction();
        SPIManager::release();

        last_write_time = millis();
        return sd_initialized;
    }

    static bool writeData(const SensorData& data) {
        if (!sd_initialized || (millis() - last_write_time < WRITE_INTERVAL)) {
            return false;
        }

        if (!SPIManager::acquireForSD(50)) {
            return false; // Не смогли захватить SPI
        }

        // Запись данных
        SPIManager::getSPI().beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE0));
        
        if (data_file) {
            data_file.write((const uint8_t*)&data, sizeof(data));
            if (millis() - last_write_time > 5000) { // Каждые 5 сек сбрасываем
                data_file.flush();
            }
        }
        
        SPIManager::getSPI().endTransaction();
        SPIManager::release();

        last_write_time = millis();
        return true;
    }

    static void end() {
        if (data_file) {
            data_file.close();
        }
        sd_initialized = false;
    }
};

// Инициализация статических переменных
uint32_t SDHandler::last_write_time = 0;
File SDHandler::data_file;
bool SDHandler::sd_initialized = false;

        //⚡ ОПТИМИЗАЦИИ ПРОИЗВОДИТЕЛЬНОСТИ
        //1. Буферизация данных SD карты
        //
// Вместо записи каждого сэмпла по отдельности
class SDHandlerBuffer {
private:
    static const int BUFFER_SIZE = 512;
    static uint8_t write_buffer[BUFFER_SIZE];
    static int buffer_index;
    
    static bool writeBuffered(const SensorData& data) {
        if (buffer_index + sizeof(data) > BUFFER_SIZE) {
            if (!flushBuffer()) return false;
        }
        memcpy(&write_buffer[buffer_index], &data, sizeof(data));
        buffer_index += sizeof(data);
        return true;
    }
    
static bool flushBuffer() {
    if (buffer_index > 0) {
        // ... ваш код записи ...
        buffer_index = 0;
        return true; // ← ДОБАВЬТЕ return
    }
    return false; // ← ДОБАВЬТЕ return
}

};

