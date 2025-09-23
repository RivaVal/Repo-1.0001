
// Config.h
#pragma once
//=======================================
//  Шаг 1: Создаем файл Config.h
//=======================================

#include <Arduino.h>
#include <driver/ledc.h>
#include "CommonTypes.h"  // ← ДОБАВИТЬ ЭТУ СТРОКУ

// ================== КОНФИГУРАЦИЯ ПИНОВ ==================
// Пины модуля E49
constexpr uint8_t E49_PIN_RX = 16;
constexpr uint8_t E49_PIN_TX = 17;
constexpr uint8_t E49_PIN_M0 = 4;
constexpr uint8_t E49_PIN_M1 = 21;
constexpr uint8_t E49_PIN_AUX = 5;

// SPI пины
constexpr uint8_t SPI_ICM_CS  = 22;
constexpr uint8_t SPI_SD_CS   = 33;
constexpr uint8_t SPI_ICM_INT = 32;
constexpr uint8_t LED_PIN = 2;

// SPI пины по умолчанию
constexpr uint8_t VSPI_SCLK = 18;
constexpr uint8_t VSPI_MISO = 19;
constexpr uint8_t VSPI_MOSI = 23;

// Конфигурация сервоприводов
constexpr int servoPins[] = {12, 13, 14, 27, 26};
constexpr ledc_channel_t servoChannels[] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4
};

// ================== ПАРАМЕТРЫ СВЯЗИ E49 ==================
constexpr uint32_t E49_BAUDRATE = 9600;
constexpr uint8_t E49_ADDRESS_H = 1;
constexpr uint8_t E49_ADDRESS_L = 101;
constexpr uint8_t E49_CHANNEL = 0x17;

// ================== ТАЙМАУТЫ И ИНТЕРВАЛЫ ==================
constexpr uint16_t AUX_TIMEOUT_MS = 300;
constexpr uint16_t ACK_TIMEOUT_MS = 500;
constexpr uint16_t RECEIVE_TIMEOUT_MS = 1000;
constexpr uint32_t RECONNECT_INTERVAL_MS = 2000;
constexpr uint32_t CONNECTION_TIMEOUT_MS = 5000;
constexpr uint8_t MAX_RETRIES = 3;

constexpr uint32_t LED_BLINK_INTERVAL_MS = 100;
constexpr uint32_t STATS_INTERVAL_MS = 15000;

// ================== ПАРАМЕТРЫ ПАКЕТОВ ==================
constexpr uint8_t CRC8_POLYNOMIAL = 0x07;
constexpr uint8_t PACKET_PREAMBLE_1 = 0xAA;
constexpr uint8_t PACKET_PREAMBLE_2 = 0x55;
constexpr uint8_t ACK_PREAMBLE_1 = 0x55;
constexpr uint8_t ACK_PREAMBLE_2 = 0xAA;
constexpr uint8_t COM_SETALL_MASK = 0b00000001;

// ================== РАЗМЕРЫ СТРУКТУР ==================
constexpr uint8_t DATA_COM_SET_SIZE = 19; // sizeof(DataComSet_t)
constexpr uint8_t ACK_PACKET_SIZE = 11;   // sizeof(AckPacket_t)

// ================== СИСТЕМНЫЕ НАСТРОЙКИ ==================
constexpr uint32_t STATUS_UPDATE_INTERVAL = 5000;
constexpr uint32_t ICM_READ_INTERVAL = 20;
constexpr uint32_t SD_WRITE_INTERVAL = 100;
constexpr uint32_t RADIO_PROCESS_INTERVAL = 50;
constexpr uint32_t ERROR_CHECK_INTERVAL = 1000;

//===========================================================

extern int servoAngles[5];
extern int servoTargetAngles[5];

        // GlobalVars.cpp:
        //      #include "GlobalVars.h"
int angles[5] = {90, 90, 90, 90, 0};
int targetAngles[5] = {90, 90, 90, 90, 0};
