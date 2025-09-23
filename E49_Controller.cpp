


//====================================================================
//	3. Файл E49_Controller.cpp
//====================================================================
//  3. Обновленный E49_Controller.cpp
//
#include "E49_Controller.h"
#include <SD.h>
#include <SPI.h>

// Таблица для быстрого расчета CRC8
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


E49_Controller::E49_Controller(bool isSender, HardwareSerial* serial) :
    _isSender(isSender),
    _debugEnabled(false),
    _ledState(false),
    _lastLedToggle(0),
    _internalState(InternalState::IDLE),
    _stateStartTime(0),
    _currentRetry(0),
    _lastResetTime(0),
    _lastSaveTime(0)
{
    memset(&_stats, 0, sizeof(_stats));
    memset(&_currentTxPacket, 0, sizeof(_currentTxPacket));
    memset(&_currentRxPacket, 0, sizeof(_currentRxPacket));
    memset(&_currentAckPacket, 0, sizeof(_currentAckPacket));
    
    // Инициализация serial порта
    serial->begin(E49_BAUDRATE, SERIAL_8N1, E49_PIN_RX, E49_PIN_TX);
    //  _transceiver = new EBYTE(serial, E49_PIN_M0, E49_PIN_M1, E49_PIN_AUX);
    _transceiver = new EBYTE(serial, E49_PIN_M0, E49_PIN_M1, E49_PIN_AUX);
    _lastActivityTime = millis();
    _lastReconnectAttempt = millis();
}

E49_Controller::~E49_Controller() {
    if (_transceiver != nullptr) {
        delete _transceiver;
        _transceiver = nullptr;
    }
}

// E49_Controller.cpp - добавьте после конструктора:
EbyteStatus E49_Controller::reinitialize() {
    if (_debugEnabled) {
        Serial.println("Reinitializing E49 module...");
    }
    
    // Сброс модуля
    resetModule();
    delay(100);
    
    // Повторная инициализация
    if (!_transceiver->init()) {
        return EbyteStatus::ERROR_INIT;
    }
    
    // Настройка параметров
    _transceiver->SetParityBit(PB_8N1);
    _transceiver->SetAddressH(E49_ADDRESS_H);
    _transceiver->SetAddressL(E49_ADDRESS_L);
    _transceiver->SetChannel(E49_CHANNEL);
    _transceiver->SetAirDataRate(ADR_19200);
    _transceiver->SetUARTBaudRate(UDR_9600);
    
    _transceiver->SaveParameters(PERMANENT);
    _lastSaveTime = millis();
    
    return EbyteStatus::SUCCESS;
}


void E49_Controller::changeState(InternalState newState) {
    if (_debugEnabled) {
        const char* stateNames[] = {
            "IDLE", "WAITING_FOR_AUX", "SENDING_DATA", "WAITING_FOR_ACK",
            "RECEIVING_DATA", "PROCESSING_DATA", "SENDING_ACK", "RESETTING", "ERROR"
        };
        
        Serial.printf("📻 [%lu] Radio FSM: %s -> %s (Retry: %d)\n", 
                    millis(),
                    stateNames[static_cast<int>(_internalState)],
                    stateNames[static_cast<int>(newState)],
                    _currentRetry);
    }
    _internalState = newState;
    _stateStartTime = millis();
}


EbyteStatus E49_Controller::init() {
    //    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Инициализация модуля
    if (!_transceiver->init()) {
        if (_debugEnabled) Serial.println("E49 module initialization failed");
        return EbyteStatus::ERROR_INIT;
    }
    
    // Настройка параметров модуля
    _transceiver->SetParityBit(PB_8N1);
    _transceiver->SetAddressH(E49_ADDRESS_H);
    _transceiver->SetAddressL(E49_ADDRESS_L);
    _transceiver->SetChannel(E49_CHANNEL);
    _transceiver->SetAirDataRate(ADR_19200);
    _transceiver->SetUARTBaudRate(UDR_9600);

    // Сохранение параметров (неблокирующее)
    _transceiver->SaveParameters(PERMANENT);
    _lastSaveTime = millis();
    
    if (_debugEnabled) {
        Serial.println("E49 module initialization started");
    }
    changeState(InternalState::IDLE); // Убираем INITIALIZING
    return EbyteStatus::SUCCESS;

}

EbyteStatus E49_Controller::waitForAux(uint32_t timeout) {
    uint32_t startTime = millis();
    
    while (millis() - startTime < timeout) {
        if (digitalRead(E49_PIN_AUX) == HIGH) {
            return EbyteStatus::SUCCESS;
        }
        
        // Неблокирующая задержка
        delay(1);
        yield();
        
        // Обновление LED индикации
        updateLED();
    }
    
    _stats.aux_timeouts++;
    return EbyteStatus::ERROR_AUX_TIMEOUT;
}

uint8_t E49_Controller::calculateCRC8(const uint8_t* data, size_t length) const {
    uint8_t crc = 0;
    
    // Оптимизированный расчет CRC8 с использованием таблицы
    for (size_t i = 0; i < length; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    
    return crc;
}

void E49_Controller::process() {
    updateLED();
    

    switch (_internalState) {
        case InternalState::IDLE:
            if (!_isSender) {
                // Приемник ожидает данные
                if (_transceiver->available()) {
                    changeState(InternalState::RECEIVING_DATA);
                }
            }
            break;
            
        case InternalState::WAITING_FOR_AUX:
            if (digitalRead(E49_PIN_AUX) == HIGH) {
                changeState(InternalState::SENDING_DATA);
            } else if (millis() - _stateStartTime > AUX_TIMEOUT_MS) {
                _stats.aux_timeouts++;
                if (++_currentRetry < MAX_RETRIES) {
                    changeState(InternalState::WAITING_FOR_AUX);
                } else {
                    changeState(InternalState::ERROR);
                }
            }
            break;
            
        case InternalState::SENDING_DATA:
            if (_transceiver->SendStruct(&_currentTxPacket, sizeof(_currentTxPacket))) {
                _stats.tx_packets++;
                _lastActivityTime = millis();
                
                BitMask mask(_currentTxPacket.comSetAll);
                if (mask.getBit(0)) {
                    changeState(InternalState::WAITING_FOR_ACK);
                } else {
                    changeState(InternalState::IDLE);
                }
            } else {
                _stats.tx_errors++;
                if (++_currentRetry < MAX_RETRIES) {
                    changeState(InternalState::WAITING_FOR_AUX);
                } else {
                    changeState(InternalState::ERROR);
                }
            }
            break;
            
        case InternalState::WAITING_FOR_ACK:
            {
                AckPacket_t ack;
                EbyteStatus status = receiveAck(ack);
                
                if (status == EbyteStatus::SUCCESS) {
                    if (ack.packet_id == _currentTxPacket.packet_id) {
                        changeState(InternalState::IDLE);
                    }
                } else if (status != EbyteStatus::ERROR_NO_NEW_DATA) {
                    if (++_currentRetry < MAX_RETRIES) {
                        changeState(InternalState::WAITING_FOR_AUX);
                    } else {
                        changeState(InternalState::ERROR);
                    }
                }
                
                if (millis() - _stateStartTime > ACK_TIMEOUT_MS) {
                    _stats.receive_timeouts++;
                    if (++_currentRetry < MAX_RETRIES) {
                        changeState(InternalState::WAITING_FOR_AUX);
                    } else {
                        changeState(InternalState::ERROR);
                    }
                }
            }
            break;
            
        case InternalState::RECEIVING_DATA:
            if (_transceiver->GetStruct(&_currentRxPacket, sizeof(_currentRxPacket))) {
                // Проверка преамбулы
                if (_currentRxPacket.preamble[0] != PACKET_PREAMBLE_1 || 
                    _currentRxPacket.preamble[1] != PACKET_PREAMBLE_2) {
                    _stats.preamble_errors++;
                    changeState(InternalState::IDLE);
                    break;
                }
                
                // Проверка CRC
                uint8_t crc = _currentRxPacket.crc8;
                _currentRxPacket.crc8 = 0;
                uint8_t calculated_crc = calculateCRC8((uint8_t*)&_currentRxPacket, sizeof(_currentRxPacket) -1);
                
                if (crc != calculated_crc) {
                    _stats.crc_errors++;
                    changeState(InternalState::IDLE);
                    break;
                }
                
                _stats.rx_packets++;
                _lastActivityTime = millis();
                
                BitMask mask(_currentRxPacket.comSetAll);
                if (mask.getBit(0)) {
                    changeState(InternalState::SENDING_ACK);
                } else {
                    changeState(InternalState::IDLE);
                }
                
                if (_debugEnabled) {
                    Serial.print("Received packet ID: ");
                    Serial.println(_currentRxPacket.packet_id);
                }
            } else {
                changeState(InternalState::IDLE);
            }
            break;
            
        case InternalState::SENDING_ACK:
            {
                EbyteStatus status = sendAck(_currentRxPacket.packet_id, 1);
                if (status == EbyteStatus::SUCCESS) {
                    changeState(InternalState::IDLE);
                } else {
                    if (++_currentRetry < MAX_RETRIES) {
                        changeState(InternalState::SENDING_ACK);
                    } else {
                        changeState(InternalState::ERROR);
                    }
                }
            }
            break;
            
        case InternalState::ERROR:
            // Обработка ошибок
            handleConnectionRecovery();
            break;
            
        default:
            changeState(InternalState::IDLE);
            break;
    }
    
    // Проверка соединения
    handleConnectionRecovery();
}

EbyteStatus E49_Controller::sendAck(uint16_t packet_id, uint8_t status) {
    AckPacket_t ack;
    ack.preamble[0] = ACK_PREAMBLE_1;
    ack.preamble[1] = ACK_PREAMBLE_2;
    ack.packet_id = packet_id;
    ack.timestamp = millis();
    ack.status = status;
    ack.crc8 = calculateCRC8((uint8_t*)&ack, sizeof(ack) - 1);
    
    EbyteStatus auxStatus = waitForAux();
    if (auxStatus != EbyteStatus::SUCCESS) {
        return auxStatus;
    }
    
    if (_transceiver->SendStruct(&ack, sizeof(ack))) {
        return EbyteStatus::SUCCESS;
    }
    
    return EbyteStatus::ERROR_SEND;
}

EbyteStatus E49_Controller::receiveAck(AckPacket_t& ack) {
    if (!_transceiver->available()) {
        return EbyteStatus::ERROR_NO_NEW_DATA;
    }
    
    if (_transceiver->GetStruct(&ack, sizeof(ack))) {
        // Проверка преамбулы
        if (ack.preamble[0] != ACK_PREAMBLE_1 || ack.preamble[1] != ACK_PREAMBLE_2) {
            _stats.preamble_errors++;
            return EbyteStatus::ERROR_INVALID_DATA;
        }
        
        // Проверка CRC
        uint8_t crc = ack.crc8;
        ack.crc8 = 0;
        uint8_t calculated_crc = calculateCRC8((uint8_t*)&ack, sizeof(ack) -1);
        
        if (crc != calculated_crc) {
            _stats.crc_errors++;
            return EbyteStatus::ERROR_CRC;
        }
        
        return EbyteStatus::SUCCESS;
    }
    
    return EbyteStatus::ERROR_NO_NEW_DATA;
}

void E49_Controller::handleConnectionRecovery() {
    if (millis() - _lastActivityTime > CONNECTION_TIMEOUT_MS) {
        _stats.connection_lost_events++;
        
        if (millis() - _lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
            _lastReconnectAttempt = millis();
            
            if (attemptRecovery()) {
                _stats.reconnects++;
                _lastActivityTime = millis();
                changeState(InternalState::IDLE);
                
                if (_debugEnabled) {
                    Serial.println("Connection recovered successfully");
                }
            } else if (_debugEnabled) {
                Serial.println("Connection recovery failed");
            }
        }
    }
}

bool E49_Controller::attemptRecovery() {
    if (_debugEnabled) {
        Serial.println("Attempting module recovery...");
    }
    
    // Сброс модуля
    resetModule();
    delay(200);
    
    // Повторная инициализация
    if (reinitialize() != EbyteStatus::SUCCESS) {
        return false;
    }
    
    // Проверка готовности модуля
    if (waitForAux(500) != EbyteStatus::SUCCESS) {
        return false;
    }
    
    return testCommunication();
}

bool E49_Controller::testCommunication() {
    if (_isSender) {
        DataComSet_t testPacket = createTestPacket();
        testPacket.comSetAll = COM_SETALL_MASK;
        
        EbyteStatus status = startSendingData(testPacket);
        if (status != EbyteStatus::SUCCESS) return false;
        
        // Ожидание завершения отправки
        uint32_t startTime = millis();
        while (_internalState != InternalState::IDLE && 
               (millis() - startTime) < 2000) {
            process();
            delay(10);
        }
        
        return _internalState == InternalState::IDLE;
    } else {
        // Для приемника проверяем готовность AUX
        return (waitForAux(100) == EbyteStatus::SUCCESS);
    }
}

void E49_Controller::resetModule() {
    if (_debugEnabled) Serial.println("Resetting module...");
    
    _transceiver->Reset();
    delay(100);
    _stats.resets++;
    _lastActivityTime = millis();
}

void E49_Controller::updateLED() {
    if (millis() - _lastLedToggle > LED_BLINK_INTERVAL_MS) {
        _ledState = !_ledState;
        digitalWrite(LED_PIN, _ledState ? HIGH : LOW);
        _lastLedToggle = millis();
    }
}

DataComSet_t E49_Controller::createTestPacket() const {
    DataComSet_t packet;
    packet.preamble[0] = PACKET_PREAMBLE_1;
    packet.preamble[1] = PACKET_PREAMBLE_2;
    packet.packet_id = 0xFFFF;
    packet.comUp = 127;
    packet.comLeft = 127;
    packet.comThrottle = 1500;
    packet.comParashut = 0;
    packet.timestamp = millis();
    packet.comSetAll = 0;
    packet.crc8 = calculateCRC8((uint8_t*)&packet, sizeof(packet) - 1);
    
    return packet;
}

EbyteStatus E49_Controller::startSendingData(const DataComSet_t& data) {
    if (!_isSender) return EbyteStatus::ERROR_BUSY;
    if (_internalState != InternalState::IDLE) return EbyteStatus::ERROR_BUSY;
    
    _currentTxPacket = data;
    _currentTxPacket.preamble[0] = PACKET_PREAMBLE_1;
    _currentTxPacket.preamble[1] = PACKET_PREAMBLE_2;
    _currentTxPacket.crc8 = calculateCRC8((uint8_t*)&_currentTxPacket, sizeof(_currentTxPacket) - 1);
    
    _currentRetry = 0;
    changeState(InternalState::WAITING_FOR_AUX);
    
    return EbyteStatus::SUCCESS;
}

void E49_Controller::printStatus(EbyteStatus status) const {
    switch(status) {
        case EbyteStatus::SUCCESS: Serial.println("Success"); break;
        case EbyteStatus::ERROR_AUX_TIMEOUT: Serial.println("AUX timeout"); break;
        case EbyteStatus::ERROR_SEND: Serial.println("Send error"); break;
        case EbyteStatus::ERROR_RECEIVE: Serial.println("Receive error"); break;
        case EbyteStatus::ERROR_CRC: Serial.println("CRC error"); break;
        case EbyteStatus::ERROR_INVALID_DATA: Serial.println("Invalid data"); break;
        case EbyteStatus::ERROR_NO_NEW_DATA: Serial.println("No new data"); break;
        case EbyteStatus::ERROR_CONNECTION_LOST: Serial.println("Connection lost"); break;
        case EbyteStatus::ERROR_NOT_READY: Serial.println("Not ready"); break;
        case EbyteStatus::ERROR_BUSY: Serial.println("Busy"); break;
        default: Serial.println("Unknown error"); break;
    }
}

void E49_Controller::printStats() const {
    Serial.println("=== E49 Module Statistics ===");
    Serial.print("TX packets: "); Serial.println(_stats.tx_packets);
    Serial.print("RX packets: "); Serial.println(_stats.rx_packets);
    Serial.print("CRC errors: "); Serial.println(_stats.crc_errors);
    Serial.print("TX errors: "); Serial.println(_stats.tx_errors);
    Serial.print("RX errors: "); Serial.println(_stats.rx_errors);
    Serial.print("Resets: "); Serial.println(_stats.resets);
    Serial.print("Reconnects: "); Serial.println(_stats.reconnects);
    Serial.print("AUX timeouts: "); Serial.println(_stats.aux_timeouts);
    Serial.print("Connection lost events: "); Serial.println(_stats.connection_lost_events);
    Serial.println("============================");
}

void E49_Controller::enableDebug(bool enable) {
    _debugEnabled = enable;
}

void E49_Controller::forceRecovery() {
    if (_debugEnabled) {
        Serial.println("Forced recovery initiated");
    }
    _lastReconnectAttempt = 0;
    _lastActivityTime = millis() - CONNECTION_TIMEOUT_MS - 1;
    handleConnectionRecovery();
}

bool E49_Controller::isConnectionLost() const {
    return (millis() - _lastActivityTime) > CONNECTION_TIMEOUT_MS;
}

//====================================================================

