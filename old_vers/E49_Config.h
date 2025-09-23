

//===========================================
//  Полная реализация с исправлениями:
//  1. Обновленный E49_Config.h
//===========================================

#pragma once 


#ifndef E49_CONFIG_H
#define E49_CONFIG_H

#include <Arduino.h>
#include <driver/ledc.h>

// Структура передаваемых данных
#pragma pack(push, 1)
typedef struct {
    uint8_t  preamble[2];    // 0xAA 0x55
    uint16_t packet_id;      // Счетчик пакетов
    uint8_t  comUp;          // 0-255
    uint8_t  comLeft;        // 0-255
    uint16_t comThrottle;    // 1000-2000
    uint8_t  comParashut;    // 0/1
    uint32_t timestamp;      // Время из millis()
    uint8_t  comSetAll;      // Битовая маска
    uint8_t  crc8;           // Контрольная сумма
} DataComSet_t;

// Структура подтверждения передачи
typedef struct {
    uint8_t preamble[2];        // 0x55 0xAA
    uint16_t packet_id;         // ID подтверждаемого пакета
    uint32_t timestamp;         // Время отправки пакета
    uint8_t status;             // Статус получения
    uint8_t crc8;               // Контрольная сумма
} AckPacket_t;

// Структура для данных сенсора ICM-20948
typedef struct {
    uint32_t timestamp;
    float accel[3];      // x, y, z акселерометр (g)
    float gyro[3];       // x, y, z гироскоп (dps)
    float mag[3];        // x, y, z магнитометр (uT)
    float euler[3];      // roll, pitch, yaw углы (градусы)
    float quat[4];       // кватернион
    uint8_t status;      // статус DMP
} SensorData;
#pragma pack(pop)

// Состояния системы
enum class SystemState {
    BOOT,           // Загрузка
    INITIALIZING,   // Инициализация
    STANDBY,        // Ожидание
    ACTIVE,         // Активная работа
    ERROR,          // Ошибка
    RECOVERY        // Восстановление
};

// Статус операций модуля E49
enum class EbyteStatus {
    SUCCESS,
    ERROR_INIT,
    ERROR_SEND,
    ERROR_RECEIVE,
    ERROR_CRC,
    ERROR_AUX_TIMEOUT,
    ERROR_NO_NEW_DATA,
    ERROR_INVALID_DATA,
    ERROR_INVALID_PREAMBLE,
    ERROR_BUSY,
    ERROR_NOT_READY,
    ERROR_CONNECTION_LOST
};

// Конфигурация пинов
static const uint8_t E49_PIN_RX = 16;
static const uint8_t E49_PIN_TX = 17;
static const uint8_t E49_PIN_M0 = 4;
static const uint8_t E49_PIN_M1 = 21;
static const uint8_t E49_PIN_AUX = 5;
static const uint8_t SPI_ICM_CS  = 22;
static const uint8_t SPI_SD_CS   = 33;
static const uint8_t SPI_ICM_INT = 32;
static const uint8_t LED_PIN = 2;

// SPI пины по умолчанию
static const uint8_t VSPI_SCLK = 18;
static const uint8_t VSPI_MISO = 19;
static const uint8_t VSPI_MOSI = 23;

// Конфигурация сервоприводов
static const int servoPins[] = {12, 13, 14, 27, 26};
static const ledc_channel_t servoChannels[] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4
};

// Параметры связи E49
static const uint32_t E49_BAUDRATE = 9600;
static const uint8_t E49_ADDRESS_H = 1;
static const uint8_t E49_ADDRESS_L = 101;
static const uint8_t E49_CHANNEL = 0x17;

// Таймауты
static const uint16_t AUX_TIMEOUT_MS = 300;
static const uint16_t ACK_TIMEOUT_MS = 500;
static const uint16_t RECEIVE_TIMEOUT_MS = 1000;
static const uint32_t RECONNECT_INTERVAL_MS = 2000;
static const uint32_t CONNECTION_TIMEOUT_MS = 5000;
static const uint8_t MAX_RETRIES = 3;
// Добавляем в секцию таймаутов:
constexpr uint32_t MODULE_RESET_DELAY_MS = 100;
constexpr uint32_t PARAMETER_SAVE_DELAY_MS = 100;
constexpr uint32_t SPI_ACQUIRE_TIMEOUT_MS = 50;

// Интервалы
static const uint32_t LED_BLINK_INTERVAL_MS = 100;
static const uint32_t STATS_INTERVAL_MS = 15000;

// CRC8 полином
static const uint8_t CRC8_POLYNOMIAL = 0x07;

// Преамбула пакета
static const uint8_t PACKET_PREAMBLE_1 = 0xAA;
static const uint8_t PACKET_PREAMBLE_2 = 0x55;
static const uint8_t ACK_PREAMBLE_1 = 0x55;
static const uint8_t ACK_PREAMBLE_2 = 0xAA;
static const uint8_t COM_SETALL_MASK = 0b00000001;

// Размеры структур
static constexpr uint8_t DATA_COM_SET_SIZE = sizeof(DataComSet_t);
static constexpr uint8_t ACK_PACKET_SIZE = sizeof(AckPacket_t);

// Статистика работы модуля
typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t crc_errors;
    uint32_t tx_errors;
    uint32_t rx_errors;
    int16_t last_rssi;
    uint32_t resets;
    uint32_t reconnects;
    uint32_t aux_timeouts;
    uint32_t busy_errors;
    uint32_t not_ready_errors;
    uint32_t connection_lost_events;
    uint32_t receive_timeouts;
    uint16_t preamble_errors;
} ModuleStats_t;

// Класс для работы с битовой маской
//================================================================
// Класс для работы с битовой маской
 class BitMask {
private:
    uint8_t mask;
public:
    BitMask() : mask(0) {}
    explicit BitMask(uint8_t initialMask) : mask(initialMask) {} // Добавить этот конструктор
    
    void setBit(uint8_t bit, bool value) {
        if (bit > 7) return;
        if (value) {
            mask |= (1 << bit);
        } else {
            mask &= ~(1 << bit);
        }
    }
    
    bool getBit(uint8_t bit) const {
        if (bit > 7) return false;
        return (mask & (1 << bit)) != 0;
    }

    void clearBit(uint8_t bit) {
        mask &= ~(1 << bit); 
    }

    bool checkBit(uint8_t bit) const { 
        return (mask & (1 << bit)) != 0;
    }
    
    void printBinary() const {
        for (int i = 7; i >= 0; i--) {
            Serial.print(getBit(i) ? "1" : "0");
        }
    }

    uint8_t getMask() const { 
        return mask; 
    }

    void setMask(uint8_t newMask) { 
        mask = newMask; 
    }
};

// Добавить в Config.h
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING, 
    ERROR,
    CRITICAL
};

// Вместо функции добавляем макросы для логирования:
#define LOG_DEBUG(format, ...)    Serial.printf("[DEBUG] " format "\n", ##__VA_ARGS__)
#define LOG_INFO(format, ...)     Serial.printf("[INFO] " format "\n", ##__VA_ARGS__)
#define LOG_WARNING(format, ...)  Serial.printf("[WARNING] " format "\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...)    Serial.printf("[ERROR] " format "\n", ##__VA_ARGS__)
#define LOG_CRITICAL(format, ...) Serial.printf("[CRITICAL] " format "\n", ##__VA_ARGS__)

// Или если нужна полноценная функция:
#ifdef ENABLE_LOGGING
void log(LogLevel level, const char* format, ...) {
    static LogLevel currentLogLevel = LogLevel::INFO;
    static const char* levelNames[] = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"};
    
    if (static_cast<int>(level) >= static_cast<int>(currentLogLevel)) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        Serial.printf("[%s] %s\n", levelNames[static_cast<int>(level)], buffer);
    }
}
#else
#define log(level, format, ...)
#endif

/*
// В классах добавить:
void log(LogLevel level, const char* format, ...) {
    if (level >= _currentLogLevel) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        Serial.printf("[%lu] %s: %s\n", millis(), logLevelToString(level), buffer);
    }
}
*/

#endif // E49_CONFIG_H

