



//====================================================================
//	2. Файл E49_Controller.h
//====================================================================
//  2. Обновленный E49_Controller.h
//

#ifndef E49_CONTROLLER_H
#define E49_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <EBYTE.h>
//  #include "Config.h"
#include "Config.h"  // ← ИЗМЕНИТЬ С E49_Config.h на Config.h


class E49_Controller {
public:
    // Состояния конечного автомата
    enum class InternalState {
        IDLE,
        WAITING_FOR_AUX,
        SENDING_DATA,
        WAITING_FOR_ACK,
        RECEIVING_DATA,
        PROCESSING_DATA,
        SENDING_ACK,
        RESETTING,
        ERROR
    };

    E49_Controller(bool isSender, HardwareSerial* serial = &Serial2);
    ~E49_Controller();
    
    // Удаляем конструктор копирования и присваивания
    E49_Controller(const E49_Controller&) = delete;
    E49_Controller& operator=(const E49_Controller&) = delete;

    EbyteStatus init();
    EbyteStatus sendData(const DataComSet_t& data);
    EbyteStatus receiveData(DataComSet_t& data);
    EbyteStatus checkConnection();
    EbyteStatus reinitialize();
    void process();
    
    // Неблокирующие методы
    EbyteStatus startSendingData(const DataComSet_t& data);
    EbyteStatus startReceivingData();
    
    // Вспомогательные методы
    DataComSet_t createTestPacket() const;
    void printStats() const;
    void printStatus(EbyteStatus status) const;
    void enableDebug(bool enable);
    void forceRecovery();
    
    // Геттеры
    InternalState getInternalState() const { return _internalState; }
    bool isConnectionLost() const;
    const ModuleStats_t& getStats() const { return _stats; }
    
private:
    // Внутренние методы
    uint8_t calculateCRC8(const uint8_t* data, size_t length) const;
    EbyteStatus waitForAux(uint32_t timeout = AUX_TIMEOUT_MS);
    EbyteStatus sendAck(uint16_t packet_id, uint8_t status);
    EbyteStatus receiveAck(AckPacket_t& ack);
    void resetModule();
    bool attemptRecovery();
    bool testCommunication();
    void handleConnectionRecovery();
    void updateLED();
    void changeState(InternalState newState);
    
    // Поля класса
    EBYTE* _transceiver;
    bool _isSender;
    bool _debugEnabled;
    ModuleStats_t _stats;
    uint32_t _lastActivityTime;
    uint32_t _lastReconnectAttempt;
    bool _ledState;
    uint32_t _lastLedToggle;
    
    // Конечный автомат
    InternalState _internalState;
    uint32_t _stateStartTime;
    DataComSet_t _currentTxPacket;
    DataComSet_t _currentRxPacket;
    AckPacket_t _currentAckPacket;
    uint8_t _currentRetry;
    
    // ... существующие поля
    uint32_t _lastResetTime;
    uint32_t _lastSaveTime;    
};

#endif // E49_CONTROLLER_H

