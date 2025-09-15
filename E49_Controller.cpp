


//====================================================================
//	3. Файл E49_Controller.cpp
//====================================================================

#include "E49_Controller.h"
#include <SD.h>
#include <SPI.h>
#include "E49_Config.h"

// Таблица для быстрого расчета CRC8
//  static const uint8_t CRC8_TABLE[256] = {
const uint8_t crc8_table[256] = {
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

E49_Controller::E49_Controller(bool isSender, HardwareSerial* serial) : 
    _isSender(isSender), 
    _debugEnabled(false),
    _lastActivityTime(millis()),  // Инициализировать текущим временем
    _lastReconnectAttempt(millis()), // Инициализировать текущим временем
//    _lastActivityTime(0),
//    _lastReconnectAttempt(0),
    _ledState(false),
    _lastLedToggle(0) {
    
    memset(&_stats, 0, sizeof(_stats));
    _transceiver = new EBYTE(serial, E49_PIN_M0, E49_PIN_M1, E49_PIN_AUX);
}

E49_Controller::~E49_Controller() {
    delete _transceiver;
}

ebyteStatus E49_Controller::init() {
                        //  pin Mode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Инициализация SD-карты (если нужно)
   // if (!SD.begin()) {
   //     if (_debugEnabled) Serial.println("SD card initialization failed");
   // }
    
    bool dd = _transceiver->init();
    Serial.print("_transceiver->init() = [");
    Serial.print(dd);
    Serial.println("]");
    
    // Настройка модуля
    _transceiver->SetParityBit(PB_8N1);    
    _transceiver->SetAddressH(E49_ADDRESS_H);
    _transceiver->SetAddressL(E49_ADDRESS_L);
    _transceiver->SetChannel(E49_CHANNEL);
    _transceiver->SetAirDataRate(ADR_19200) ; 
    _transceiver->SetUARTBaudRate(UDR_9600) ; 
    //_transceiver->SetSpeed(E49_SPEED);          // UDR_9600 0b011		// 9600 baud default
    //_transceiver->SetOptions(E49_OPTIONS);      // ADR_19200 0b100		// 19.200 baud 

    
    // Исправленная строка: используем константу из библиотеки EBYTE
    //  if (_transceiver->SaveParameters(PERMANENT) != 1) {
    _transceiver->SaveParameters(PERMANENT); // != 1) {
                        //if (_debugEnabled) Serial.println("Failed to save parameters");
                        //    return EBYTE_ERROR_INIT;
                    // }
    delay(100);
    //_transceiver->PrintParameters() ;
            Serial.print("_transceiver->PrintParameters(): ");    
    // Проверка связи
    // ebyteStatus status = checkConnection(); //Связи еще нет, что проверять??
    // УБРАТЬ ошибочную проверку - она всегда "падает"
            //Serial.print("Initial connection check failed: ");
            //Serial.println("status");
    //if (status != EBYTE_SUCCESS) {
        if (_debugEnabled) {
            //Serial.print("Initial connection check failed: ");
            //Serial.println("status");
        }
        //return status;
    //}
    
    if (_debugEnabled) Serial.println("E49 module initialized successfully");
    return EBYTE_SUCCESS;
}



ebyteStatus E49_Controller::waitForAux(uint32_t timeout) const {
    uint32_t startTime = millis();
    while (digitalRead(E49_PIN_AUX) == LOW) {
        if (millis() - startTime > timeout) {
            // Исправление: снимаем const с метода
            const_cast<E49_Controller*>(this)->_stats.aux_timeouts++;
            return EBYTE_ERROR_AUX_TIMEOUT;
        }
        delay(1);
    }
    return EBYTE_SUCCESS;
}

/* Устаревшая версия
ebyteStatus E49_Controller::sendData(const DataComSet_t& data) {
    if (!_isSender) return EBYTE_ERROR_BUSY;
    
    DataComSet_t packet = data;
    // Установка преамбулы
    packet.preamble[0] = 0xAA;
    packet.preamble[1] = 0x55;
    
    // Расчет CRC
    packet.crc8 = calculateCRC8((uint8_t*)&packet, sizeof(packet) - 1);
    
    // Отправка данных
    ebyteStatus status = waitForAux();
    if (status != EBYTE_SUCCESS) {
        _stats.tx_errors++;
        return status;
    }
    
    if (_transceiver->SendStruct(&packet, sizeof(packet)) != 1) {
        _stats.tx_errors++;
        return EBYTE_ERROR_SEND;
    }
    
    _stats.tx_packets++;
    _lastActivityTime = millis();
    
    // Если установлен первый бит в comSetall, ждем подтверждения
    BitMask mask(packet.comSetAll);
    if (mask.checkBit(0)) {
        AckPacket_t ack;
        uint32_t startTime = millis();
        
        while (millis() - startTime < RECEIVE_TIMEOUT_MS) {
            status = receiveAck(ack);
            if (status == EBYTE_SUCCESS) {
                if (ack.packet_id == packet.packet_id) {
                    if (_debugEnabled) printAck(ack);
                    return EBYTE_SUCCESS;
                }
            } else if (status != EBYTE_ERROR_NO_NEW_DATA) {
                break;
            }
            delay(10);
        }
        
        _stats.receive_timeouts++;
        return EBYTE_ERROR_RECEIVE;
    }
    
    return EBYTE_SUCCESS;
}


//2. Улучшенный метод sendData() с повторными попытками:
ebyteStatus E49_Controller::sendData(const DataComSet_t& data) {
    if (!_isSender) return EBYTE_ERROR_BUSY;
    
//    const uint8_t MAX_RETRIES = 3;
    uint8_t retryCount = 0;
    
    while (retryCount < MAX_RETRIES) {
        DataComSet_t packet = data;
        packet.preamble[0] = 0xAA;
        packet.preamble[1] = 0x55;

        // Добавить все данные в структуру packet
        packet.crc8 = calculateCRC8((uint8_t*)&packet, sizeof(packet) - 1);
        
        ebyteStatus status = waitForAux(100);
        if (status != EBYTE_SUCCESS) {
            _stats.tx_errors++;
            retryCount++;
            delay(50 * retryCount); // Увеличиваем задержку при повторных попытках
            continue;
        }
        
        if (_transceiver->SendStruct(&packet, sizeof(packet)) != 1) {
            _stats.tx_errors++;
            retryCount++;
            delay(50 * retryCount);
            continue;
        }
        
        _stats.tx_packets++;
        _lastActivityTime = millis();
        
        // Обработка ACK
        BitMask mask(packet.comSetAll);
        if (mask.checkBit(0)) {
            AckPacket_t ack;
            uint32_t startTime = millis();
            
            while ((millis() - startTime) < RECEIVE_TIMEOUT_MS) {
                status = receiveAck(ack);
                if (status == EBYTE_SUCCESS) {
                    if (ack.packet_id == packet.packet_id) {
                        if (_debugEnabled) printAck(ack);
                        return EBYTE_SUCCESS;
                    }
                }
                delay(10);
            }
            
            _stats.receive_timeouts++;
            retryCount++;
            continue;
        }
        
        return EBYTE_SUCCESS;
    }
    
    // Все попытки исчерпаны
    if (_debugEnabled) {
        Serial.println("All send attempts failed");
    }
    return EBYTE_ERROR_SEND;
}
*/

//  4. Исправить метод sendData()
ebyteStatus E49_Controller::sendData(const DataComSet_t& data) {
    if (!_isSender) return EBYTE_ERROR_BUSY;
    
    const uint8_t MAX_RETRIES = 3; // Добавить если нет в заголовке
    uint8_t retryCount = 0;
    
    while (retryCount < MAX_RETRIES) {
        // ... подготовка пакета ...
        DataComSet_t packet = data;
        packet.preamble[0] = 0xAA;
        packet.preamble[1] = 0x55;
        packet.crc8 = calculateCRC8((uint8_t*)&packet, sizeof(packet) - 1);
        
        ebyteStatus status = waitForAux(100); // Добавить timeout
        if (status != EBYTE_SUCCESS) {
            if (_debugEnabled) {
                Serial.print("AUX timeout, retry: ");
                Serial.println(retryCount);
            }
            _stats.tx_errors++;
            retryCount++;
            delay(50 * retryCount);
            continue;
        }
        // ... остальной код ...
                
        if (_transceiver->SendStruct(&packet, sizeof(packet)) != 1) {
            _stats.tx_errors++;
            retryCount++;
            delay(50 * retryCount);
            continue;
        }
        
        _stats.tx_packets++;
        _lastActivityTime = millis();
        
        // Обработка ACK
        BitMask mask(packet.comSetAll);
        if (mask.checkBit(0)) {
            AckPacket_t ack;
            uint32_t startTime = millis();
            
            while ((millis() - startTime) < RECEIVE_TIMEOUT_MS) {
                status = receiveAck(ack);
                if (status == EBYTE_SUCCESS) {
                    if (ack.packet_id == packet.packet_id) {
                        if (_debugEnabled) printAck(ack);
                        return EBYTE_SUCCESS;
                    }
                }
                delay(10);
            }
            
            _stats.receive_timeouts++;
            retryCount++;
            continue;
        }
        
        return EBYTE_SUCCESS;
    }
    
    // Все попытки исчерпаны
    if (_debugEnabled) {
        Serial.println("All send attempts failed");
    }
    return EBYTE_ERROR_SEND;
}





ebyteStatus E49_Controller::receiveData(DataComSet_t& data) {
    if (_isSender) return EBYTE_ERROR_BUSY;
    
    ebyteStatus status = waitForAux();
    if (status != EBYTE_SUCCESS) {
        _stats.rx_errors++;
        return status;
    }
    
    if (_transceiver->available()) {
        if (_transceiver->GetStruct(&data, sizeof(data))) {
            // Проверка преамбулы
            if (data.preamble[0] != 0xAA || data.preamble[1] != 0x55) {
                _stats.preamble_errors++;
                return EBYTE_ERROR_INVALID_DATA;
            }
            
            // Проверка CRC
            uint8_t crc = data.crc8;
            data.crc8 = 0;
            uint8_t calculated_crc = calculateCRC8((uint8_t*)&data, sizeof(data));
            
            if (crc != calculated_crc) {
                _stats.crc_errors++;
                return EBYTE_ERROR_CRC;
            }
            
            _stats.rx_packets++;
            _lastActivityTime = millis();
            
            // Если установлен первый бит в comSetall, отправляем подтверждение
            BitMask mask(data.comSetAll);
            if (mask.checkBit(0)) {
                status = sendAck(data.packet_id, EBYTE_SUCCESS);
                if (status != EBYTE_SUCCESS) {
                    return status;
                }
            }
            
            if (_debugEnabled) printStruct(data);
            return EBYTE_SUCCESS;
        }
    }
    
    return EBYTE_ERROR_NO_NEW_DATA;
}

/* Устаревшая версия кода     
ebyteStatus E49_Controller::checkConnection() {
    if ( (millis() - _lastActivityTime ) > CONNECTION_TIMEOUT_MS) {
        _stats.connection_lost_events++;
        resetModule();
        _lastReconnectAttempt = millis();
        return EBYTE_ERROR_CONNECTION_LOST;
    }
    return EBYTE_SUCCESS;
}
*/

// Добавьте метод принудительного восстановления:
void E49_Controller::forceRecovery() {
    if (_debugEnabled) {
        Serial.println("Forced recovery initiated");
    }
    _lastReconnectAttempt = 0; // Немедленная попытка восстановления
    _lastActivityTime = millis() - CONNECTION_TIMEOUT_MS - 1; // Принудительная проверка соединения
    
    handleConnectionRecovery();
}


// Изменить checkConnection():
ebyteStatus E49_Controller::checkConnection() {
    if ((millis() - _lastActivityTime) > CONNECTION_TIMEOUT_MS) {
        _stats.connection_lost_events++;
        return EBYTE_ERROR_CONNECTION_LOST;
    }
    return EBYTE_SUCCESS;
}


/* Устаревшая версия
void E49_Controller::process() {
    updateLED();
    
    if (!_isSender) {
        DataComSet_t data;
        ebyteStatus status = receiveData(data);
        if (status == EBYTE_SUCCESS) {
            // Обработка полученных данных
        }
    }
    
    // Проверка соединения
    if (millis() - _lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
        checkConnection();
    }
}
*/

//Улучшенный код с автоматическим восстановлением:
//1. Улучшенный метод process():
void E49_Controller::process() {
    updateLED();
    
    if (!_isSender) {
        DataComSet_t data;
        ebyteStatus status = receiveData(data);
        
        if (status == EBYTE_SUCCESS) {
            // Обработка полученных данных
            if (_debugEnabled) {
                Serial.print("Successfully received packet ID: ");
                Serial.println(data.packet_id);
            }
        } else if (status != EBYTE_ERROR_NO_NEW_DATA) {
            if (_debugEnabled) {
                Serial.print("Receive error: ");
                Serial.println(status);
            }
        }
    }
    // Автоматическое восстановление связи
    handleConnectionRecovery();
}

// Новый метод для обработки восстановления связи
void E49_Controller::handleConnectionRecovery() {
    ebyteStatus connStatus = checkConnection();
    
    if (connStatus == EBYTE_ERROR_CONNECTION_LOST) {
        if (_debugEnabled) {
            Serial.println("Connection lost detected, attempting recovery...");
        }
        
        // Попытка восстановления
        if (millis() - _lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
            _lastReconnectAttempt = millis();
            
            if (attemptRecovery()) {
                if (_debugEnabled) {
                    Serial.println("Connection recovered successfully!");
                }
                _stats.reconnects++;
                _lastActivityTime = millis(); // Сброс таймера активности
            } else {
                if (_debugEnabled) {
                    Serial.println("Recovery attempt failed");
                }
            }
        }
    }
}

/*  
// Метод попытки восстановления
bool E49_Controller::attemptRecovery() {
    if (_debugEnabled) {
        Serial.println("Attempting module recovery...");
    }
    
    // 1. Мягкий сброс
    resetModule();
    delay(200);
    
    // 2. Повторная инициализация
    ebyteStatus status = reinitialize();
    
    // 3. Проверка работоспособности
    if (status == EBYTE_SUCCESS) {
        return testCommunication();
    }
    
    return false;
}
*/

// Метод попытки восстановления
bool E49_Controller::attemptRecovery() {
    if (_debugEnabled) {
        Serial.println("Attempting module recovery...");
    }
    
    // 1. Мягкий сброс
    resetModule();
    delay(200);

    // 2. Повторная инициализация
    if (reinitialize() != EBYTE_SUCCESS) {
        if (_debugEnabled) {
            Serial.println("Reinitialization failed");
        }
        return false;
    }
    
    // 3. Краткая проверка готовности модуля
    if (waitForAux(500) != EBYTE_SUCCESS) {
        if (_debugEnabled) {
            Serial.println("Module not ready after reinitialization");
        }
        return false;
    }
    
    if (_debugEnabled) {
        Serial.println("Module recovery successful");
    }
    return true;
}

/*      
// Метод тестирования связи
bool E49_Controller::testCommunication() {
    if (_isSender) {
        // Для передатчика: попытка отправить тестовый пакет
        DataComSet_t testPacket = createTestPacket();
        return (sendData(testPacket) == EBYTE_SUCCESS);
    } else {
        // Для приемника: ожидание данных в течение короткого времени
        uint32_t startTime = millis();
        while (millis() - startTime < TEST_COMMUNICATION_TIMEOUT_MS) {
            DataComSet_t data;
            if (receiveData(data) == EBYTE_SUCCESS) {
                return true;
            }
            delay(10);
        }
        return false;
    }
}
*/

// Упростить testCommunication() - убрать бесконечные циклы
bool E49_Controller::testCommunication() {
    // Простая проверка готовности модуля
    return (waitForAux(100) == EBYTE_SUCCESS);
}



// Создание тестового пакета
DataComSet_t E49_Controller::createTestPacket() const {
    DataComSet_t packet;
    packet.preamble[0] = 0xAA;
    packet.preamble[1] = 0x55;
    packet.packet_id = 0xFFFF;
    packet.comUp = 127;
    packet.comLeft = 127;
    packet.comThrottle = 1501;
    packet.comParashut = 0;
    packet.timestamp = millis();
    packet.comSetAll = 0b00000001; // Запрос ACK
    packet.crc8 = calculateCRC8((uint8_t*)&packet, sizeof(packet) - 1);
    
    return packet;
}


// Метод повторной инициализации
ebyteStatus E49_Controller::reinitialize() {
    if (_debugEnabled) {
        Serial.println("Reinitializing module...");
    }
    
    // Сброс модуля
    _transceiver->Reset();
    delay(200);
    
    // Повторная настройка параметров
    _transceiver->SetAddressH(E49_ADDRESS_H);
    _transceiver->SetAddressL(E49_ADDRESS_L);
    _transceiver->SetChannel(E49_CHANNEL);
    _transceiver->SetAirDataRate(ADR_19200);
    _transceiver->SetUARTBaudRate(UDR_9600);
    _transceiver->SetParityBit(PB_8N1);
    
    // Сохранение параметров
    _transceiver->SaveParameters(PERMANENT);
    delay(100);
    
    // Проверка готовности модуля
    if (waitForAux(1000) != EBYTE_SUCCESS) {
        if (_debugEnabled) {
            Serial.println("Module not ready after reinitialization");
        }
        return EBYTE_ERROR_NOT_READY;
    }
    return EBYTE_SUCCESS;
}






void E49_Controller::updateLED() {
    if (millis() - _lastLedToggle > LED_BLINK_INTERVAL_MS) {
        _ledState = !_ledState;
        digitalWrite(LED_PIN, _ledState ? HIGH : LOW);
        _lastLedToggle = millis();
    }
}

uint8_t E49_Controller::calculateCRC8(const uint8_t* data, size_t length) const {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/*  
ebyteStatus E49_Controller::waitForAux(uint32_t timeout) const {
    uint32_t startTime = millis();
    while (digitalRead(E49_PIN_AUX) == LOW) {
        if (millis() - startTime > timeout) {
            _stats.aux_timeouts++;
            return EBYTE_ERROR_AUX_TIMEOUT;
        }
        delay(1);
    }
    return EBYTE_SUCCESS;
}
*/


ebyteStatus E49_Controller::sendAck(uint16_t packet_id, uint8_t status) {
    AckPacket_t ack;
    ack.preamble[0] = 0x55;
    ack.preamble[1] = 0xAA;
    ack.packet_id = packet_id;
    ack.status = status;
    ack.crc8 = calculateCRC8((uint8_t*)&ack, sizeof(ack) - 1);
    
    ebyteStatus auxStatus = waitForAux();
    if (auxStatus != EBYTE_SUCCESS) {
        return auxStatus;
    }
    
    if (_transceiver->SendStruct(&ack, sizeof(ack))) {
        return EBYTE_SUCCESS;
    }
    return EBYTE_ERROR_SEND;
}

ebyteStatus E49_Controller::receiveAck(AckPacket_t& ack) {
    if (_transceiver->available()) {
        if (_transceiver->GetStruct(&ack, sizeof(ack))) {
            // Проверка преамбулы
            if (ack.preamble[0] != 0x55 || ack.preamble[1] != 0xAA) {
                _stats.preamble_errors++;
                return EBYTE_ERROR_INVALID_DATA;
            }
            
            // Проверка CRC
            uint8_t crc = ack.crc8;
            ack.crc8 = 0;
            uint8_t calculated_crc = calculateCRC8((uint8_t*)&ack, sizeof(ack));
            
            if (crc != calculated_crc) {
                _stats.crc_errors++;
                return EBYTE_ERROR_CRC;
            }
            
            return EBYTE_SUCCESS;
        }
    }
    return EBYTE_ERROR_NO_NEW_DATA;
}


/*      
void E49_Controller::resetModule() {
    Serial.println("E49_Controller::resetModule()");
    _transceiver->Reset();
    Serial.println("E49_Controller::_transceiver->Reset()");
    delay(100);
    Serial.println("E49_Controller:init() before");
    init();
    Serial.println("E49_Controller::init() after");
    _stats.resets++;
}
*/

// Изменить resetModule():
void E49_Controller::resetModule() {
    if (_debugEnabled) Serial.println("Resetting module...");
    _transceiver->Reset();
    delay(100);
    _stats.resets++;
    _lastActivityTime = millis();  // Сбросить таймер активности
}


void E49_Controller::logToSD(const String& message) {
    File logFile = SD.open("/logs/log.txt", FILE_WRITE);
    if (logFile) {
        logFile.println(message);
        logFile.close();
    }
}

void E49_Controller::printStats() const {
    Serial.println("=== Module Statistics ===");
    Serial.print("TX packets: "); Serial.println(_stats.tx_packets);
    Serial.print("RX packets: "); Serial.println(_stats.rx_packets);
    Serial.print("CRC errors: "); Serial.println(_stats.crc_errors);
    Serial.print("TX errors: "); Serial.println(_stats.tx_errors);
    Serial.print("RX errors: "); Serial.println(_stats.rx_errors);
    Serial.print("Resets: "); Serial.println(_stats.resets);
    Serial.print("Reconnects: "); Serial.println(_stats.reconnects);
    Serial.print("Connection lost events: "); Serial.println(_stats.connection_lost_events);
    Serial.println("========================");
}

void E49_Controller::printStruct(const DataComSet_t& data) const {
    Serial.println("=== Data Structure ===");
    Serial.print("Packet ID: "); Serial.println(data.packet_id);
    Serial.print("comUp: "); Serial.println(data.comUp);
    Serial.print("comLeft: "); Serial.println(data.comLeft);
    Serial.print("comThrottle: "); Serial.println(data.comThrottle);
    Serial.print("comParashut: "); Serial.println(data.comParashut);
    Serial.print("Timestamp: "); Serial.println(data.timestamp);
    Serial.print("comSetAll: "); Serial.println(data.comSetAll, BIN);
    Serial.print("CRC: "); Serial.println(data.crc8, HEX);
    Serial.println("=====================");
}

void E49_Controller::printAck(const AckPacket_t& ack) const {
    Serial.println("=== ACK Packet ===");
    Serial.print("Packet ID: "); Serial.println(ack.packet_id);
    Serial.print("Status: "); Serial.println(ack.status);
    Serial.print("CRC: "); Serial.println(ack.crc8, HEX);
    Serial.println("================");
}

//  5. Добавить более детальную диагностику
void E49_Controller::printStatus(ebyteStatus status) const {
    switch(status) {
        case EBYTE_SUCCESS: Serial.println("Success"); break;
        case EBYTE_ERROR_AUX_TIMEOUT: Serial.println("AUX timeout"); break;
        case EBYTE_ERROR_SEND: Serial.println("Send error"); break;
        case EBYTE_ERROR_RECEIVE: Serial.println("Receive error"); break;
        case EBYTE_ERROR_CRC: Serial.println("CRC error"); break;
        case EBYTE_ERROR_INVALID_DATA: Serial.println("Invalid data"); break;
        case EBYTE_ERROR_NO_NEW_DATA: Serial.println("No new data"); break;
        case EBYTE_ERROR_CONNECTION_LOST: Serial.println("Connection lost"); break;
        case EBYTE_ERROR_NOT_READY: Serial.println("Not ready"); break;
        case EBYTE_ERROR_BUSY: Serial.println("Busy"); break;
        default: Serial.println("Unknown error"); break;
    }
}


void E49_Controller::enableDebug(bool enable) {
    _debugEnabled = enable;
}
//====================================================================

