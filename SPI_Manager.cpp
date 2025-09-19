    //
    //  2. Исправленный SPI_Manager.cpp
    //  cpp
    //
#include "SPI_Manager.h"

bool SPIManager::spi_busy = false;
uint32_t SPIManager::last_operation_time = 0;

bool SPIManager::begin() {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    pinMode(SPI_ICM_CS, OUTPUT);
    pinMode(SPI_SD_CS, OUTPUT);
    digitalWrite(SPI_ICM_CS, HIGH);
    digitalWrite(SPI_SD_CS, HIGH);
    spi_busy = false;
    return true;
}

bool SPIManager::acquireForICM(uint32_t timeout) {
    return acquireSPI(SPI_ICM_CS, timeout);
}

bool SPIManager::acquireForSD(uint32_t timeout) {
    return acquireSPI(SPI_SD_CS, timeout);
}

void SPIManager::release() {
    digitalWrite(SPI_ICM_CS, HIGH);
    digitalWrite(SPI_SD_CS, HIGH);
    spi_busy = false;
}

bool SPIManager::acquireSPI(uint8_t cs_pin, uint32_t timeout) {
    uint32_t start = millis();
    
    while (spi_busy) {
        if (millis() - start > timeout) {
            return false;
        }
        delay(1);
    }

    spi_busy = true;
    digitalWrite(SPI_ICM_CS, HIGH);
    digitalWrite(SPI_SD_CS, HIGH);
    digitalWrite(cs_pin, LOW);
    last_operation_time = millis();
    
    return true;
}
