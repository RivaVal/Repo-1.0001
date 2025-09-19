// для отлпдки__
// from source file :  Project_new_Module_001_001
// сьорка последнего файла :: ICM_29048_SD_Card_022_001_FINAL_FF.txt
// Последний проект сборка ::  sk_Reci_MCPWM_aug22_0054_72_2
//
//============================================================
//	1. E49_Config.h
//============================================================сы_
#ifndef E49_CONFIG_H
#define E49_CONFIG_H

#pragma once


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
#pragma pack(pop)

// Структура подтверждения передачи
#pragma pack(push, 1)
typedef struct {
    uint8_t preamble[2];        // 1 0x55 0xAA
    uint16_t packet_id;         // 2 ID подтверждаемого пакета
    uint32_t timestamp ;        // 4 Время отправки пакета
    uint8_t status;             // 1 Статус получения
    uint8_t crc8;               // 1 Контрольная сумма
} AckPacket_t;
        //#pragma pack(pop)

// Размеры структур
constexpr uint8_t DATA_COM_SET_SIZE =  sizeof(DataComSet_t) ;
constexpr uint8_t ACK_PACKET_SIZE = sizeof(AckPacket_t) ;

// Добавить новые определения:
#define E49_ICM_CS   22    // Новое имя для ICM
#define E49_SD_CS    33    // Новое имя для SD
#define E49_ICM_INT  32    // Interrupt pin (опционально)
// Используем стандартный VSPI
// VSPI пины по умолчанию на ESP32:
#define VSPI_SCLK   18
#define VSPI_MISO   19
#define VSPI_MOSI   23

// Конфигурация пинов для модуля E49 
const uint8_t E49_PIN_RX = 16 ;
const uint8_t E49_PIN_TX = 17 ;
const uint8_t E49_PIN_M0 = 4  ;
const uint8_t E49_PIN_M1 = 21;  // Вместо 18 - используем свободный GPIO21
const uint8_t E49_PIN_AUX = 5 ;

// Параметры связи
#define SERIAL_BAUDRATE 9600
#define E49_BAUDRATE 9600
#define E49_ADDRESS_H 1
#define E49_ADDRESS_L 101
#define E49_ADDRESS 0x0001
#define E49_CHANNEL 0x17
                //  #define E49_CHANNEL  0x11
#define E49_SPEED 0b00011011  // 115200 UART, 19200 Air
#define E49_OPTIONS 0b11100100  // Fixed + Pullup + FEC + 11dBm

// Таймауты для восстановления связи
const uint32_t RECONNECT_INTERVAL_MS = 2000;      // Интервал между попытками восстановления
const uint32_t TEST_COMMUNICATION_TIMEOUT_MS = 500; // Таймаут тестирования связи
const uint32_t MAX_RECONNECT_ATTEMPTS = 5;        // Максимальное количество попыток


//  #define AUX_TIMEOUT_MS 300         // 
//  #define COM_SETALL_MASK 0b00000001  //         // Маска данных  
//  #define RECEIVE_TIMEOUT_MS 1000      // Задержеа перед очередным получением данных

// Таймауты и интервалы
#define ACK_TIMEOUT 1000        // 1 секунда на ожидание ACK
#define RESEND_DELAY 500        // 0.5 секунды перед повторной отправкой
#define CONNECTION_CHECK_INTERVAL 10000  // Проверка соединения каждые 10 секунд
#define DATA_SEND_INTERVAL 1000  // Отправка данных каждую секунду
#define ACK_REQUEST_INTERVAL 17  // Запрашивать ACK каждые 17 отправок

// Настройки передачи
#define TRANSMIT_INTERVAL 500    //100  // Интервал отправки (мс)
const uint8_t MAX_RETRIES = 7;  // 3          // Макс. попыток повторной отправки
#define AUX_TIMEOUT_MS 300   //100       //
#define RECEIVE_TIMEOUT_MS 1000   //200
#define AUX_WAIT_TIMEOUT_MS 500   //    200
#define CONNECTION_TIMEOUT_MS 5000

// LED индикация
#define LED_PIN 2
#define LED_BLINK_INTERVAL_MS 100

// CRC8 полином
#define CRC8_POLYNOMIAL 0x07

// Преамбула пакета
const uint8_t PACKET_PREAMBLE_1 = 0xAA ;
const uint8_t PACKET_PREAMBLE_2 = 0x55 ;
const uint8_t COM_SETALL_MASK = 0b00000001 ; //  Начальная Маска данных  

#define DEBUG_LEVEL 5
#define STATS_INTERVAL_MS 15000


// Коды ошибок
#define ERR_NO_ERROR 0
#define ERR_NO_ACK 1
#define ERR_CONNECTION_LOST 2
#define ERR_SEND_FAILED 3

// Статус операций модуля E49
enum ebyteStatus {
    EBYTE_SUCCESS,
    EBYTE_ERROR_INIT,
    EBYTE_ERROR_SEND,
    EBYTE_ERROR_RECEIVE,
    EBYTE_ERROR_CRC,
    EBYTE_ERROR_AUX_TIMEOUT,
    EBYTE_ERROR_NO_NEW_DATA,
    EBYTE_ERROR_INVALID_DATA,
    EBYTE_ERROR_INVALID_PREAMBLE,
    EBYTE_ERROR_BUSY,
    EBYTE_ERROR_NOT_READY,
    EBYTE_ERROR_CONNECTION_LOST
};

// Добавьте если нет:
extern const int servoPins[5];
extern const ledc_channel_t channels[5];


// Структура c бинарными данными для сбора их из модуля ICM_20948
#pragma pack(push, 1)
struct SensorData {
  uint32_t timestamp;
  float accel[3];      // x, y, z акселерометр (g)
  float gyro[3];       // x, y, z гироскоп (dps)
  float mag[3];        // x, y, z магнитометр (uT)
  float euler[3];      // roll, pitch, yaw углы (градусы)
  float quat[4];       // кватернион
  uint8_t status;      // статус DMP
};
#pragma pack(pop)

// Статистика работы модуля E49
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
    uint16_t preamble_errors ;
} ModuleStats_t;


// Размеры буферов // Размеры структур
            //  constexpr uint8_t MAX_PACKET_SIZE =  sizeof(DataComSet_t) ;
            //  constexpr uint8_t ACK_PACKET_SIZE =  sizeof(AckPacket_t) ;

// ================== КОНСТАНТЫ ВРЕМЕННЫХ ИНТЕРВАЛОВ ==================
const uint32_t INTERVAL_EBYTE_PROCESS = 50;
const uint32_t INTERVAL_ELERON_UPDATE = 150;
const uint32_t INTERVAL_MOTOR_UPDATE = 100;
const uint32_t INTERVAL_SENSOR_READ = 100;
const uint32_t INTERVAL_STATS_DISPLAY = 5000;
const uint32_t INTERVAL_MAIN_LOOP = 150;
const uint32_t INTERVAL_CONNECTION_CHECK = 10000;

// ================== СТРУКТУРА ДЛЯ ТАЙМЕРОВ МОДУЛЕЙ ==================
typedef struct {
  uint32_t lastEbyteTime;
  uint32_t lastEleronTime;
  uint32_t lastMotorTime;
  uint32_t lastSensorTime;
  uint32_t lastStatsTime;
  uint32_t lastMainLoopTime;
  uint32_t lastConnectionCheckTime;
} ModuleTimers_t;

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


// Ключи, отладка 
typedef struct {
    bool RC_Debug;
    bool recieve_Debug;          // обратите внимание на написание! // Отладочный вывод функции init()
    bool sender_Debug;         // Отладочный вывод функции init()
    bool ebyte_Debug;
    bool mcpwm_Debug;
    bool ledc_Debug;
    bool setup_Debug;
    bool loop_Debug;
    bool diod_Debug;
    bool icm20984_Debug;
    bool eleron_Debug;           // Debug ALL eleron prog
    bool slidePot_Debug;        // Отладка для SLIDE_POT проекта
    bool throttle_Debug;         // Debug ALL Throttle prog   
    bool joystick_Debug;
    bool incomedata_Debug;
    bool processdata_Debug;     // Debug all process function
    bool cc;
 } ControllerDebug_t;
#pragma pack(pop)

//========================================================================
#endif // E49_CONFIG_H