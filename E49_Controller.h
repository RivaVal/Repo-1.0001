



//====================================================================
//	2. Файл E49_Controller.h
//====================================================================

#ifndef E49_CONTROLLER_H
#define E49_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <EBYTE.h>
#include "E49_Config.h"



/*  
class BitMask {
public:
    BitMask(uint8_t mask = 0) : _mask(mask) {}
    
    void setBit(uint8_t bit) { _mask |= (1 << bit); }
    void clearBit(uint8_t bit) { _mask &= ~(1 << bit); }
    bool checkBit(uint8_t bit) const { return (_mask & (1 << bit)) != 0; }
    uint8_t getMask() const { return _mask; }
    void setMask(uint8_t mask) { _mask = mask; }
    
private:
    uint8_t _mask;
};
*/
//       ----
 //                   class E49_Controller {
 //                   public:
 //                       E49_Controller(bool isReceiver);
 //                       ebyteStatus init();
 //                       void process();
 //                       // ... другие методы ...
 //
//----------
class E49_Controller {
public:
    E49_Controller(bool isSender, HardwareSerial* serial = &Serial2);
    ~E49_Controller();
    
    ebyteStatus init();
    ebyteStatus sendData(const DataComSet_t& data);
    ebyteStatus receiveData(DataComSet_t& data);
    ebyteStatus checkConnection();
    ebyteStatus reinitialize();
    DataComSet_t createTestPacket() const ;
    void process();
    void printStats() const;
    void printStruct(const DataComSet_t& data) const;
    void printAck(const AckPacket_t& ack) const;
    void enableDebug(bool enable);
    void updateLED();
    void forceRecovery();
    void handleConnectionRecovery();
    bool testCommunication();
    bool attemptRecovery() ;

    void printStatus(ebyteStatus status) const; // Добавьте эту строку
    // ... остальные методы ...
    bool isConnectionLost() const;
    //void forceRecovery();
    //void printStats() const;

private:
    uint8_t calculateCRC8(const uint8_t* data, size_t length) const;
    ebyteStatus waitForAux(uint32_t timeout = AUX_TIMEOUT_MS) const;
    ebyteStatus sendAck(uint16_t packet_id, uint8_t status);
    ebyteStatus receiveAck(AckPacket_t& ack);
    void resetModule();
    void logToSD(const String& message);
    
    EBYTE* _transceiver;
    bool _isSender;
    bool _debugEnabled;
    ModuleStats_t _stats;
    uint32_t _lastActivityTime;
    uint32_t _lastReconnectAttempt;
    bool _ledState;
    uint32_t _lastLedToggle;
};

#endif // E49_CONTROLLER_H
//====================================================================
