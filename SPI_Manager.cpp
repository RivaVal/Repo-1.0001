
//
//===================================================================
//  2. Исправленный SPI_Manager.cpp
//===================================================================
//
//  2. Исправленный SPI_Manager.cpp
//
#include "SPI_Manager.h"

bool SPIManager::spi_busy = false;
uint32_t SPIManager::last_operation_time = 0;
uint8_t SPIManager::current_cs_pin = 0;

bool SPIManager::begin() {
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);
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
    if (current_cs_pin != 0) {
        digitalWrite(current_cs_pin, HIGH);
        current_cs_pin = 0;
    }
    spi_busy = false;
}

bool SPIManager::acquireSPI(uint8_t cs_pin, uint32_t timeout) {
    uint32_t start = millis();
    
    while (spi_busy && (millis() - start < timeout)) {
        delay(1); // Короткая пауза, но лучше использовать vTaskDelay в FreeRTOS
    }
    
    if (spi_busy) return false;
    
    spi_busy = true;
    current_cs_pin = cs_pin;
    digitalWrite(cs_pin, LOW);
    last_operation_time = millis();
    
    return true;
}

