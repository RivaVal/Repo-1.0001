

//========================================
//  Шаг 2: Создаем файл CommonTypes.h
//========================================
// CommonTypes.h
#pragma once

#include <Arduino.h>
#include <cstdint>

// ================== СТРУКТУРЫ ДАННЫХ ==================
#pragma pack(push, 1)

// Структура передаваемых данных
struct DataComSet_t {
    uint8_t  preamble[2];    // 0xAA 0x55
    uint16_t packet_id;      // Счетчик пакетов
    uint8_t  comUp;          // 0-255
    uint8_t  comLeft;        // 0-255
    uint16_t comThrottle;    // 1000-2000
    uint8_t  comParashut;    // 0/1
    uint32_t timestamp;      // Время из millis()
    uint8_t  comSetAll;      // Битовая маска
    uint8_t  crc8;           // Контрольная сумма
};

// Структура подтверждения передачи
struct AckPacket_t {
    uint8_t preamble[2];        // 0x55 0xAA
    uint16_t packet_id;         // ID подтверждаемого пакета
    uint32_t timestamp;         // Время отправки пакета
    uint8_t status;             // Статус получения
    uint8_t crc8;               // Контрольная сумма
};

// Структура для данных сенсора ICM-20948
struct SensorData {
    uint32_t timestamp;
    float accel[3];      // x, y, z акселерометр (g)
    float gyro[3];       // x, y, z гироскоп (dps)
    float mag[3];        // x, y, z магнитометр (uT)
    float euler[3];      // roll, pitch, yaw углы (градусы)
    float quat[4];       // кватернион
    uint8_t status;      // статус DMP
};

// Статистика работы модуля
struct ModuleStats_t {
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
};

#pragma pack(pop)

// ================== ПЕРЕЧИСЛЕНИЯ ==================
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
    ERROR_PREAMBLE,
    ERROR_SEND_FAILED,
    ERROR_RECEIVE_FAILED,
    ERROR_MODULE_NOT_RESPONDING,
    ERROR_INVALID_STATE,
    ERROR_BUSY,
    ERROR_NOT_READY,
    ERROR_CONNECTION_LOST
};
/*
// Добавить более детализированные коды ошибок
enum class EbyteStatus {
    SUCCESS,
    ERROR_AUX_TIMEOUT,
    ERROR_CRC,
    ERROR_PREAMBLE,
    ERROR_SEND_FAILED,
    ERROR_RECEIVE_FAILED,
    ERROR_MODULE_NOT_RESPONDING,
    ERROR_INVALID_STATE
    // ...
};
*/

// ================== ВСПОМОГАТЕЛЬНЫЙ КЛАСС ==================
// Класс для работы с битовой маской
class BitMask {
private:
    uint8_t mask;
public:
    BitMask() : mask(0) {}
    explicit BitMask(uint8_t initialMask) : mask(initialMask) {}
    
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

    uint8_t getMask() const { return mask; }
    void setMask(uint8_t newMask) { mask = newMask; }
};

