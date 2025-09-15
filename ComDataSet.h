//
//      ComDataSet.h
//
// Собраны все протоколв обмена данными между тлчками управления
//
// Общие настройки и определения для Sender и Receiver
/*      


#ifndef COMMDATASET_H
#define COMMDATASET_H

#pragma once

#include <stdint.h>
#include <cstddef>

//===========================================================================
//	2. Конфигурационный файл (config.h)
//===========================================================================
//========================================================================
//	2. Код для Sender и Receiver
//	2.1. Общие определения и настройки
//========================================================================

// Общие настройки и определения для Sender и Receiver
    
// 
#include <Arduino.h>
#include <HardwareSerial.h>
#include <EBYTE.h>


      
// Пины подключения модуля E49
#define EBYTE_PIN_M0 4
#define EBYTE_PIN_M1 18
#define EBYTE_PIN_AUX 5
#define EBYTE_PIN_RX 16
#define EBYTE_PIN_TX 17

// Статус операций ebyte
enum ebyteStatus {
    EBYTE_SUCCESS,
    EBYTE_ERROR_INIT,
    EBYTE_ERROR_SEND,
    EBYTE_ERROR_RECEIVE,
    EBYTE_ERROR_CRC,
    EBYTE_ERROR_AUX_TIMEOUT,
    EBYTE_ERROR_NO_NEW_DATA
};

// Структура передаваемых данных
typedef struct {
    uint8_t     preamble[2];   // 0xAA 0x55
    uint16_t    packet_id;     // Счетчик пакетов
    uint8_t     comUp;         // 0-255
    uint8_t     comLeft;       // 0-255
    uint16_t    comThrottle;   // 1000-2000
    uint8_t     comParashut;   // 0/1
    uint32_t    timestamp;     // millis()
    uint8_t     comSetall;     // Битовая маска
    uint8_t     crc8;          // Контрольная сумма
} DataComSet_t;

// Статистика работы модуля
typedef struct {
    uint32_t tx_packets;        // Отправлено пакетов
    uint32_t rx_packets;        // Получено пакетов
    uint32_t crc_errors;        // Ошибок CRC
    uint32_t tx_errors;         // Ошибок передачи
    uint32_t rx_errors;         // Ошибок приема
    uint32_t last_rssi;         // Последний RSSI
    uint32_t resets;            // Количество сбросов модуля
    uint32_t reconnects;        // Количество переподключений
} ModuleStats_t;

// Таблица для расчета CRC8 (полином 0x07)
const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xD9, 0xDE, 0xD7, 0xD0, 0xC5, 0xC2, 0xCB, 0xCC, 0xE1, 0xE6, 0xEF, 0xE8, 0xFD, 0xFA, 0xF3, 0xF4,
    0x49, 0x4E, 0x47, 0x40, 0x55, 0x52, 0x5B, 0x5C, 0x71, 0x76, 0x7F, 0x78, 0x6D, 0x6A, 0x63, 0x64,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

// Функция расчета CRC8
uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}
//========================================================================
// Класс для работы с модулем E49
class E49Controller {
private:
    EBYTE* _transceiver;
    HardwareSerial* _serial;
    ModuleStats_t _stats;
    uint16_t _packet_counter;
    bool _module_initialized;
    
    // Проверка состояния AUX пина с таймаутом
    bool wait_for_aux(uint32_t timeout_ms = 200) {
        uint32_t start = millis();
        while (millis() - start < timeout_ms) {
            if (digitalRead(AUX_PIN) {
                return true;
            }
            delay(1);
        }
        return false;
    }
    
    // Сброс модуля
    void reset_module() {
        _transceiver->Reset();
        delay(50);
        _stats.resets++;
    }
    
public:
    E49Controller(HardwareSerial* serial) : 
        _serial(serial), 
        _packet_counter(0),
        _module_initialized(false) {
        memset(&_stats, 0, sizeof(_stats));
    }
    
    // Инициализация модуля
    ebyteStatus init() {
                            // pin Mode(AUX_PIN, INPUT);
                            // pin Mode(M0_PIN, OUTPUT);
                            // pin Mode(M1_PIN, OUTPUT);
        
        _transceiver = new EBYTE(_serial, M0_PIN, M1_PIN, AUX_PIN);
        
        _serial->begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
        
        if (!wait_for_aux()) {
            return EBYTE_ERROR_AUX_TIMEOUT;
        }
        
        if (!_transceiver->init()) {
            return EBYTE_ERROR_INIT;
        }
        
        // Настройки модуля (можно изменить под ваши нужды)
        _transceiver->SetMode(MODE_NORMAL);
        _transceiver->SetAirDataRate(ADR_19200);
        _transceiver->SetAddress(0x0000);
        _transceiver->SetChannel(0x04);
        _transceiver->SetTransmitPower(OPT_TP20);
        _transceiver->SetPullupMode(OPT_PULLUP);
        _transceiver->SetWORTiming(OPT_WOR_2000);
        _transceiver->SetFECMode(OPT_FEC_ENABLE);
        _transceiver->SetTransmissionMode(OPT_FIXED);
        
        _module_initialized = true;
        return EBYTE_SUCCESS;
    }
    
    // Отправка структуры данных
    ebyteStatus sendData(const DataComSet_t* data) {
        if (!_module_initialized) {
            return EBYTE_ERROR_INIT;
        }
        
        // Подготовка временной структуры для отправки
        DataComSet_t tx_data = *data;
        tx_data.preamble[0] = 0xAA;
        tx_data.preamble[1] = 0x55;
        tx_data.packet_id = _packet_counter++;
        tx_data.timestamp = millis();
        
        // Расчет CRC (исключая поле crc8 из расчета)
        tx_data.crc8 = calculate_crc8((uint8_t*)&tx_data, sizeof(DataComSet_t) - sizeof(uint8_t));
        
        // Отправка данных
        if (!wait_for_aux()) {
            reset_module();
            _stats.tx_errors++;
            return EBYTE_ERROR_AUX_TIMEOUT;
        }
        
        if (!_transceiver->sendStruct(&tx_data, sizeof(tx_data))) {
            _stats.tx_errors++;
            return EBYTE_ERROR_SEND;
        }
        
        _stats.tx_packets++;
        return EBYTE_SUCCESS;
    }
    
    // Прием структуры данных
    ebyteStatus receiveData(DataComSet_t* data, uint32_t timeout_ms = 100) {
        if (!_module_initialized) {
            return EBYTE_ERROR_INIT;
        }
        
        uint32_t start = millis();
        while (millis() - start < timeout_ms) {
            if (_transceiver->available()) {
                if (_transceiver->GetStruct(data, sizeof(DataComSet_t))) {
                    // Проверка преамбулы
                    if (data->preamble[0] != 0xAA || data->preamble[1] != 0x55) {
                        _stats.rx_errors++;
                        return EBYTE_ERROR_RECEIVE;
                    }
                    
                    // Проверка CRC
                    uint8_t received_crc = data->crc8;
                    data->crc8 = 0; // Обнуляем crc перед расчетом
                    uint8_t calculated_crc = calculate_crc8((uint8_t*)data, sizeof(DataComSet_t));
                    
                    if (received_crc != calculated_crc) {
                        _stats.crc_errors++;
                        return EBYTE_ERROR_CRC;
                    }
                    
                    _stats.rx_packets++;
                    _stats.last_rssi = _transceiver->GetRSSI();
                    return EBYTE_SUCCESS;
                }
            }
            delay(1);
        }
        
        return EBYTE_ERROR_NO_NEW_DATA;
    }
    
    // Получение статистики
    const ModuleStats_t& getStats() const {
        return _stats;
    }
    
    // Восстановление соединения
    ebyteStatus reconnect() {
        _module_initialized = false;
        _stats.reconnects++;
        return init();
    }
};
//========================================================================


        //  
//  ===========================================================================
//  Конфигурация оборудования
//  Подключение модуля E49 к ESP32
//===========================================================================
 
// Пины подключения для ESP32
#define EBYTE_PIN_RX 16   // Serial2 RX (подключить к TX EBYTE)
#define EBYTE_PIN_TX 17   // Serial2 TX (подключить к RX EBYTE)
#define EBYTE_PIN_M0 4    // Режим M0
#define EBYTE_PIN_M1 22   // Режим M1
#define EBYTE_PIN_AUX 21  // Индикатор состояния


//  Настройка проекта
//  Структура проекта
//===========================================================================
//  
// Настройки модуля E49-400T20D
#define FREQUENCY 433.0      // MHz
#define BANDWIDTH 125.0      // kHz
#define SPREADING_FACTOR 9   // SF
#define CODING_RATE 7        // CR 4/7
#define OUTPUT_POWER 20      // dBm
#define CURRENT_LIMIT 120    // mA
#define PREAMBLE_LEN 8       // Символов
#define SYNC_WORD 0x12       // Синхр. слово

// Настройки передачи
#define TRANSMIT_INTERVAL 100  // 10 раз в секунду (100 мс)
#define PACKET_PREAMBLE_1 0xAA
#define PACKET_PREAMBLE_2 0x55
#define MAX_RETRIES 3         // Макс. попыток ретрансляции
// 





//      
// Структура данных
#pragma pack(push, 1)
typedef struct {
    uint8_t preamble[2];    // 0xAA 0x55
    uint16_t packet_id;     // Счетчик пакетов
    uint8_t comUp;          // 0-255
    uint8_t comLeft;        // 0-255
    uint16_t comThrottle;   // 1000-2000
    uint8_t comParashut;    // 0/1
    uint32_t timestamp;     // millis()
    uint8_t comSetall;      // Битовая маска
    uint8_t crc8;           // Контрольная сумма
} DataComSet_t;
#pragma pack(pop)
//


// Константы
//  const uint8_t PREAMBLE[] = {0xAA, 0x55};
//  const uint32_t PACKET_INTERVAL_MS = 1000 / PACKETS_PER_SECOND;



//===========================================================================
// Настройки сети -- WiFi Сввязь
#define AP_SSID "ESP32_DRONE_Receiver"
#define AP_PASSWORD "password123"
#define AP_CHANNEL 6
//      #define WIFI_CHANNEL     6
#define LOCAL_IP         {192, 168, 4, 1}
#define GATEWAY          {192, 168, 4, 1}
#define SUBNET           {255, 255, 255, 0}
//
#define MAX_CONN 1

// Настройки передачи   // Network settings

#define PACKET_RATE 100  // 100ms = 10 пакетов в секунду
#define RECONNECT_DELAY 2000
#define WIFI_TIMEOUT 10000
#define PACKET_TIMEOUT 2000
#define SERVER_PORT 54321

// Transmission protocol
#define PACKET_RATE_MS 100    // 10 packets/sec
#define ACK_TIMEOUT_MS 500    // Wait for ACK
#define RECONNECT_DELAY_MS 2000
#define WIFI_TIMEOUT_MS 10000
#define MAX_RETRIES 3

// Константы пакета
#define PREAMBLE_1 0xAA
#define PREAMBLE_2 0x55
#define ACK_SIGNAL 0xCC

#define SERVER_IP "192.168.4.1"
#define TCP_PORT 54321
#define SERVER_PORT 54321
// 
//

//===============================================================
//	2. Сервер (AP) с расширенной диагностикой
//===============================================================
//===============================================================
//	1. Обновленная структура с CRC8
//  //#pragma pack(push, 1)
// Packet structure
typedef  struct __attribute__((packed)) {
    uint8_t preamble[2];    // 2  0xAA 0x55
    uint16_t packet_id;     // 2  Счетчик пакетов
    uint8_t comUp;          // 1  0-255
    uint8_t comLeft;        // 1  0-255
    uint16_t comThrottle;   // 2  1000-2000
    uint8_t comParashut;    // 1  0/1
    uint32_t timestamp;     // 4  millis()
    uint8_t comSetall;      // 1  Битовая маска        // uint8_t comPayload[10]; // Доп. данные
    uint8_t crc8;           // 1  Контрольная сумма
}  DataComSet_t ;           // Размер структуры 15



// Константы
const uint8_t PREAMBLE[] = {0xAA, 0x55};
//      const uint32_t PACKET_INTERVAL_MS = 1000 / PACKETS_PER_SECOND;
const size_t DATA_COM_SET_SIZE = sizeof(DataComSet_t);  // Должно быть 15 байт


// Данные по работе с управлением направления движения // крен, тангаж, рыскание
typedef  struct __attribute__((packed)) {
  float roll ;      //  крен
  float pitch ;     //  тангаж
  float yaw ;       //  рыскание

  uint8_t aux_channels[4]; // Вспомогательные Данные 
}   ICMControlData  ;



// Таблица для быстрого расчета CRC8
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

//  calculateCRC8
uint8_t calculateCRC8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}
//
#endif //   COMMDATASET_H
*/