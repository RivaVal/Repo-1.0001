


    //  3. Исправленный ICM_Handler.cpp
    //  cpp
    //  4. Исправленный ICM_Handler.cpp
    //  cpp
    //
#include "ICM_Handler.h"

// Определение статических переменных
uint32_t ICMHandler::last_read_time = 0;
          //  SensorData ICMHandler::current_data = {0};
uint32_t ICMHandler::read_count = 0;
uint32_t ICMHandler::error_count = 0;


    //  3. Обновим ICM_Handler.cpp
    //  cpp
    //
// Правильная инициализация SensorData
SensorData ICMHandler::current_data = {
    .timestamp = 0,
    .accel = {0, 0, 0},
    .gyro = {0, 0, 0},
    .mag = {0, 0, 0},
    .euler = {0, 0, 0},
    .quat = {0, 0, 0, 0},
    .status = 0
};


bool ICMHandler::begin() {
    if (!SPIManager::acquireForICM(10)) {
        error_count++;
        return false;
    }

    bool success = false;
    SPIClass& spi = SPIManager::getSPI();
    
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    try {
        // Инициализация ICM
        digitalWrite(SPI_ICM_CS, LOW);
        spi.transfer(0x80 | 0x00); // WHO_AM_I
        uint8_t id = spi.transfer(0);
        digitalWrite(SPI_ICM_CS, HIGH);
        
        success = (id == 0xEA);
    } catch (...) {
        success = false;
    }
    
    spi.endTransaction();
    SPIManager::release();
    
    if (!success) error_count++;
    return success;
}

bool ICMHandler::readData() {
    if (millis() - last_read_time < READ_INTERVAL) {
        return false;
    }

    if (!SPIManager::acquireForICM(5)) {
        error_count++;
        return false;
    }

    SPIClass& spi = SPIManager::getSPI();
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    
    digitalWrite(SPI_ICM_CS, LOW);
    // Чтение данных ICM
    spi.transfer(0x80 | 0x06); // ACCEL_XOUT_H
    current_data.accel[0] = (spi.transfer(0) << 8) | spi.transfer(0);
    current_data.accel[1] = (spi.transfer(0) << 8) | spi.transfer(0);
    current_data.accel[2] = (spi.transfer(0) << 8) | spi.transfer(0);
    digitalWrite(SPI_ICM_CS, HIGH);
    
    spi.endTransaction();
    SPIManager::release();

    current_data.timestamp = millis();
    last_read_time = current_data.timestamp;
    read_count++;
    
    return true;
}

const SensorData& ICMHandler::getData() {
    return current_data;
}

bool ICMHandler::isDataFresh() {
    return (millis() - last_read_time) < (READ_INTERVAL + 5);
}

uint32_t ICMHandler::getReadCount() {
    return read_count;
}

uint32_t ICMHandler::getErrorCount() {
    return error_count;
}