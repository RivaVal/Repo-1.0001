

    //======================================
    //  3. Исправленный ICM_Handler.cpp
    //======================================
    //  4. Исправленный ICM_Handler.cpp
    //
    //  6. ICM_Handler.cpp - Полная реализация с буфером
    //

#include "ICM_Handler.h"

// Инициализация статических переменных
uint32_t ICMHandler::last_read_time = 0;
SensorData ICMHandler::current_data = {
    .timestamp = 0,
    .accel = {0, 0, 0},
    .gyro = {0, 0, 0},
    .mag = {0, 0, 0},
    .euler = {0, 0, 0},
    .quat = {0, 0, 0, 0},
    .status = 0
};
uint32_t ICMHandler::read_count = 0;
uint32_t ICMHandler::error_count = 0;

// Буферные переменные
SensorData ICMHandler::data_buffer[ICMHandler::BUFFER_SIZE];
size_t ICMHandler::buffer_index = 0;
bool ICMHandler::buffer_overflow = false;

    /*  
// ICM_Handler.cpp
bool ICMHandler::begin() {
    if (!SPIManager::acquireForICM(10)) {
        return false;
    }

    // Быстрая инициализация без ожидания
    SPIClass& spi = SPIManager::getSPI();
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    
    digitalWrite(SPI_ICM_CS, LOW);
    spi.transfer(0x80 | 0x00);
    uint8_t id = spi.transfer(0);
    digitalWrite(SPI_ICM_CS, HIGH);
    
    spi.endTransaction();
    SPIManager::release();
    
    bool success = (id == 0xEA);
    if (success) {
        Serial.println("ICM initialized");
    }
    
    return success;
}
*/

    
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
        if (success) {
            Serial.println("✅ ICM-20948 initialized successfully");
        } else {
            Serial.printf("❌ ICM-20948 init failed. ID: 0x%02X\n", id);
        }
    } catch (...) {
        success = false;
        Serial.println("❌ Exception during ICM initialization");
    }
    

    spi.endTransaction();
    SPIManager::release();
    
    if (!success) error_count++;
    
    // Инициализация буфера
    clearBuffer();
    
    return success;
}

bool ICMHandler::readData() {
    uint32_t current_time = millis();
    
    // Защита от переполнения millis()
    if (current_time - last_read_time < READ_INTERVAL) {
        return false;
    }

    //if ((uint32_t)(current_time - last_read_time) < READ_INTERVAL) {
    //    return false;
    //}

    if (!SPIManager::acquireForICM(5)) {
        error_count++;
        Serial.println("⚠️ Failed to acquire SPI for ICM");
        return false;
    }

    bool read_success = true;
    SPIClass& spi = SPIManager::getSPI();
    
    try {
        spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
        
        // Чтение акселерометра
        digitalWrite(SPI_ICM_CS, LOW);
        spi.transfer(0x80 | 0x2D); // ACCEL_XOUT_H
        current_data.accel[0] = (spi.transfer(0) << 8) | spi.transfer(0);
        current_data.accel[1] = (spi.transfer(0) << 8) | spi.transfer(0);
        current_data.accel[2] = (spi.transfer(0) << 8) | spi.transfer(0);
        digitalWrite(SPI_ICM_CS, HIGH);
        
       // Чтение гироскопа (добавьте аналогично)
        // Чтение гироскопа
        digitalWrite(SPI_ICM_CS, LOW);
        spi.transfer(0x80 | 0x33); // GYRO_XOUT_H
        current_data.gyro[0] = ((int16_t)(spi.transfer(0) << 8) | spi.transfer(0)) * 250.0 / 32768.0;
        current_data.gyro[1] = ((int16_t)(spi.transfer(0) << 8) | spi.transfer(0)) * 250.0 / 32768.0;
        current_data.gyro[2] = ((int16_t)(spi.transfer(0) << 8) | spi.transfer(0)) * 250.0 / 32768.0;
        digitalWrite(SPI_ICM_CS, HIGH);

        // Чтение магнитометра (требует дополнительной инициализации)
        digitalWrite(SPI_ICM_CS, LOW);
        spi.transfer(0x80 | 0x11); // MAG_XOUT_L
        int16_t mag_x = (spi.transfer(0) << 8) | spi.transfer(0);
        int16_t mag_y = (spi.transfer(0) << 8) | spi.transfer(0);
        int16_t mag_z = (spi.transfer(0) << 8) | spi.transfer(0);
        digitalWrite(SPI_ICM_CS, HIGH);

        current_data.mag[0] = mag_x * 0.15; // Конвертация в µT
        current_data.mag[1] = mag_y * 0.15;
        current_data.mag[2] = mag_z * 0.15;
         // Чтение магнитометра (добавьте аналогично)
        
        spi.endTransaction();
        
    } catch (...) {
        read_success = false;
        Serial.println("❌ Exception during ICM data reading");
    }
    
    SPIManager::release();

    if (read_success) {
        current_data.timestamp = current_time;
        last_read_time = current_time;
        read_count++;
        
        // Добавление данных в буфер
        addToBuffer(current_data);
        
        // Периодический отчет о состоянии буфера
        if (read_count % 50 == 0) {
            Serial.printf("📊 ICM Buffer: %d/%d (%.1f%%)", 
                         buffer_index, BUFFER_SIZE, getBufferUsage());
            if (buffer_overflow) Serial.print(" [OVERFLOW]");
            Serial.println();
        }
        if (read_success && read_count % 20 == 0) {
                Serial.printf("📊 [%lu] IMU: A(%.2f,%.2f,%.2f) G(%.2f,%.2f,%.2f) M(%.2f,%.2f,%.2f)\n", millis(),
                 current_data.accel[0], current_data.accel[1], current_data.accel[2],
                 current_data.gyro[0], current_data.gyro[1], current_data.gyro[2],
                 current_data.mag[0], current_data.mag[1], current_data.mag[2]);
        }
    } else {
        error_count++;
    }
    
    return read_success;
}

bool ICMHandler::addToBuffer(const SensorData& data) {
    if (buffer_index < BUFFER_SIZE) {
        data_buffer[buffer_index++] = data;
        return true;
    } else {
        buffer_overflow = true;
        Serial.println("⚠️ ICM buffer overflow! Data lost.");
        return false;
    }
}

const SensorData& ICMHandler::getData() {
    return current_data;
}

bool ICMHandler::isDataFresh() {
    uint32_t current_time = millis();
    return (uint32_t)(current_time - last_read_time) < (READ_INTERVAL + 5);
}

uint32_t ICMHandler::getReadCount() {
    return read_count;
}

uint32_t ICMHandler::getErrorCount() {
    return error_count;
}


//=================================================================================
    /*
    //  7. Интеграция с SDHandler - Пример использования буфера
    //  cpp

// В основном коде или в SDHandler
void processICMData() {
    if (ICMHandler::getBufferSize() > 0) {
        const SensorData* buffer = ICMHandler::getBuffer();
        size_t buffer_size = ICMHandler::getBufferSize();
        
        // Пакетная запись в SD
        for (size_t i = 0; i < buffer_size; i++) {
            SDHandler::writeData(buffer[i]);
        }
        
        // Очистка буфера после записи
        ICMHandler::clearBuffer();
        
        Serial.printf("💾 Written %d samples from ICM buffer to SD\n", buffer_size);
    }
}

// Или в loop()
void loop() {
    // Чтение данных с ICM
    ICMHandler::readData();
    
    // Периодическая запись буфера на SD (например, каждые 2 секунды)
    static uint32_t last_buffer_flush = 0;
    if (millis() - last_buffer_flush > 2000) {
        processICMData();
        last_buffer_flush = millis();
    }
}
    */
//========================================================================================