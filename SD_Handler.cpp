


    //===================================================
    //  6. SD_Handler.cpp - Реализация с буферизацией
    //===================================================
    //
    //  cpp

#include "SD_Handler.h"
#include <SPI.h>

// Инициализация статических переменных
uint32_t SDHandler::last_write_time = 0;
uint32_t SDHandler::last_flush_time = 0;
File SDHandler::data_file;
bool SDHandler::sd_initialized = false;
uint8_t SDHandler::write_buffer[SDHandler::BUFFER_SIZE];
size_t SDHandler::buffer_index = 0;
uint32_t SDHandler::write_count = 0;
uint32_t SDHandler::error_count = 0;

bool SDHandler::begin() {
    if (!SPIManager::acquireForSD(100)) {
        Serial.println("❌ Failed to acquire SPI for SD");
        return false;
    }

    SPIManager::getSPI().beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE0));
    sd_initialized = SD.begin(E49_SD_CS);
    
    if (sd_initialized) {
        // Создаем директорию для данных
        if (!SD.exists("/data")) {
            SD.mkdir("/data");
        }
        
        // Создаем файл с timestamp
        char filename[32];
        snprintf(filename, sizeof(filename), "/data/data_%lu.bin", millis());
        data_file = SD.open(filename, FILE_WRITE);
        
        if (!data_file) {
            Serial.println("❌ Failed to open data file");
            sd_initialized = false;
        } else {
            Serial.println("✅ SD card and file initialized");
            Serial.printf("📊 Free space: %llu bytes\n", getFreeSpace());
        }
    }
    
    SPIManager::getSPI().endTransaction();
    SPIManager::release();

    last_write_time = millis();
    last_flush_time = millis();
    buffer_index = 0;
    
    return sd_initialized;
}

bool SDHandler::writeData(const SensorData& data) {
    if (!sd_initialized) return false;
    
    // Проверка свободного места
    if (!checkFreeSpace()) {
        Serial.println("⚠️ Low disk space, stopping writes");
        sd_initialized = false;
        return false;
    }
    
    uint32_t current_time = millis();
    if ((uint32_t)(current_time - last_write_time) < WRITE_INTERVAL) {
        return false;
    }

    // Буферизация данных
    if (buffer_index + sizeof(data) > BUFFER_SIZE) {
        if (!flushBuffer()) {
            error_count++;
            return false;
        }
    }
    
    memcpy(&write_buffer[buffer_index], &data, sizeof(data));
    buffer_index += sizeof(data);
    write_count++;
    
    // Авто-сброс буфера каждые 5 секунд или при заполнении
    if (current_time - last_flush_time > 5000 || buffer_index >= BUFFER_SIZE * 0.8) {
        if (!flushBuffer()) {
            error_count++;
            return false;
        }
    }
    
    last_write_time = current_time;
    return true;
}

bool SDHandler::flushBuffer() {
    if (buffer_index == 0) return true;
    
    if (!SPIManager::acquireForSD(100)) {
        Serial.println("⚠️ Could not acquire SPI for flush");
        return false;
    }

    bool success = false;
    SPIManager::getSPI().beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE0));
    
    if (data_file) {
        size_t bytes_written = data_file.write(write_buffer, buffer_index);
        success = (bytes_written == buffer_index);
        
        if (success) {
            data_file.flush();
            Serial.printf("💾 Flushed %d bytes to SD\n", buffer_index);
        } else {
            Serial.printf("❌ Write failed: %d/%d bytes\n", bytes_written, buffer_index);
        }
    }
    
    SPIManager::getSPI().endTransaction();
    SPIManager::release();
    
    buffer_index = 0;
    last_flush_time = millis();
    return success;
}

bool SDHandler::checkFreeSpace() {
    if (!sd_initialized) return false;
    
    uint64_t free_space = getFreeSpace();
    if (free_space < MIN_FREE_SPACE) {
        Serial.printf("⚠️ Low disk space: %llu bytes left\n", free_space);
        return false;
    }
    
    return true;
}

uint64_t SDHandler::getFreeSpace() {
    if (!sd_initialized) return 0;
    
    if (!SPIManager::acquireForSD(50)) return 0;
    
    SPIManager::getSPI().beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    uint64_t total = SD.totalBytes();
    uint64_t used = SD.usedBytes();
    SPIManager::getSPI().endTransaction();
    SPIManager::release();
    
    return (total > used) ? (total - used) : 0;
}

void SDHandler::end() {
    // Сброс оставшихся данных в буфере
    if (buffer_index > 0) {
        flushBuffer();
    }
    
    if (data_file) {
        data_file.close();
        Serial.println("✅ Data file closed");
    }
    
    sd_initialized = false;
    buffer_index = 0;
}

float SDHandler::getSuccessRate() {
    return (write_count > 0) ? (100.0f * (write_count - error_count) / write_count) : 100.0f;
}