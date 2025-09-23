
/**================================================
 * @file ESP32_Air_Plane_0001.ino
 * @brief Основной модуль системы управления БПЛА
 * @version 2.0
 * @date 2024
 *================================================
 * Система включает:
 * - Радиомодуль E49 для приема команд
 * - IMU ICM-20948 для навигации
 * - Управление сервоприводами элеронов
 * - SD карту для логирования данных
 * - Систему мониторинга и восстановления
 */

#include <Arduino.h>
#include <SPI.h>
#include "SPI_Manager.h"
#include "ICM_Handler.h"
#include "SD_Handler.h"
#include "Eleron_Controller.h"
#include "E49_Controller.h"
#include "Config.h"

// ================== СИСТЕМНЫЕ НАСТРОЙКИ ==================


// ================== СИСТЕМНЫЕ ПЕРЕМЕННЫЕ ==================

SystemState systemState = SystemState::BOOT;
SystemState previousState = SystemState::BOOT;
uint32_t systemStartTime = 0;
uint32_t errorCount = 0;
uint32_t lastStatusPrint = 0;

// ================== ЭКЗЕМПЛЯРЫ МОДУЛЕЙ ==================
E49_Controller radioReceiver(false, &Serial2);

// ================== ПРОТОТИПЫ ФУНКЦИЙ ==================
void initializeSystem();
void initializeAllPins();
void handleSystemState();
void processSensors();
void handleRadioCommunication();
void controlOutputs();
void logTelemetry();
void updateSystemStatus();
void handleEmergency();
void systemRecovery();
void printStatusReport();
void checkSystemHealth();

// ================== ОСНОВНЫЕ ФУНКЦИИ ==================

//  1. Добавьте функцию инициализации пинов в setup():
void initializeAllPins() {
  // Пины EBYTE
  pinMode(E49_PIN_M0, OUTPUT);
  pinMode(E49_PIN_M1, OUTPUT);
  pinMode(E49_PIN_AUX, INPUT);

  // SPI пины
  pinMode(VSPI_SCLK, OUTPUT);
  pinMode(VSPI_MISO, INPUT);
  pinMode(VSPI_MOSI, OUTPUT);
  pinMode(SPI_ICM_CS, OUTPUT);
  pinMode(SPI_SD_CS, OUTPUT);
  // Дополнительные пины
  pinMode(SPI_ICM_INT, INPUT);

  // Пины сервоприводов
  for (int i = 0; i < 5; i++) {
    pinMode(servoPins[i], OUTPUT);
  }

  // Светодиод
  pinMode(LED_PIN, OUTPUT);


  Serial.println("All pins initialized");
}  // end  initializeAllPins()




void setup() {
    initializeAllPins();

    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, E49_PIN_RX, E49_PIN_TX);
    
            // handleRadioCommunication();
    Serial.println("\n=== СИСТЕМА УПРАВЛЕНИЯ БПЛА ===");
    Serial.println("Версия 2.0 - Запуск...");
    
    initializeSystem();
    systemStartTime = millis();

    delay(1000);
}

void loop() {
    static uint32_t loopStart = millis();
    uint32_t currentTime = millis();
    handleRadioCommunication() ;
    handleSystemState();
    processSensors();
    controlOutputs();
    logTelemetry();
    updateSystemStatus();
    checkSystemHealth();
    
    // Стабильная задержка цикла
    uint32_t processingTime = millis() - currentTime;
    if (processingTime < 10) {
        delay(10 - processingTime);
    }

    // Добавить проверку времени выполнения
    uint32_t loopTime = millis() - loopStart;
    if (loopTime > 50) {  // Предупреждение если цикл > 50мс
        Serial.printf("⚠️ Long loop time: %lumS\n", loopTime);
    }

}

// ================== РЕАЛИЗАЦИЯ ФУНКЦИЙ ==================
void initializeSystem() {
    systemState = SystemState::INITIALIZING;
    Serial.println("🔄 Инициализация модулей...");
    
    // bool allOk = true;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
    // Добавьте таймаут инициализации
    uint32_t initStart = millis();
    while (!ICMHandler::begin() && (( millis() - initStart) < 10000)) {
        Serial.println("⚠️ Retrying ICM initialization...");
        delay(1000);
    }
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // 1. Инициализация SPI
    SPIManager::begin();
    Serial.println("SPI менеджер: OK");
    
    // 2. Инициализация IMU
    if (ICMHandler::begin()) {
        Serial.println("ICM-20948: OK");
    } else {
        Serial.println("ICM-20948: ERROR");
        systemState = SystemState::ERROR;
        return;
    }
    
    // 3. Инициализация SD карты
    if (SDHandler::begin()) {
        Serial.println("SD карта: OK");
    } else {
        Serial.println("SD карта: WARNING");
    }
    
    // 4. Инициализация элеронов
    EleronController::begin();
    Serial.println("Сервоприводы: OK");
    
    // 5. Инициализация радиомодуля
    if (radioReceiver.init() == EbyteStatus::SUCCESS) {
        Serial.println("Радиомодуль: OK");
    } else {
        Serial.println("Радиомодуль: ERROR");
        systemState = SystemState::ERROR;
        return;
    }
    
    systemState = SystemState::STANDBY;
    Serial.println("=== СИСТЕМА ГОТОВА ===");
}

void handleSystemState() {
        //  static uint32_t lastStateChange = 0;
    
    switch (systemState) {
        case SystemState::STANDBY:
            // Мигание LED в режиме ожидания
            digitalWrite(LED_PIN, millis() % 1000 < 500);
            break;
            
        case SystemState::ACTIVE:
            // Постоянное свечение в активном режиме
            digitalWrite(LED_PIN, HIGH);
            break;
            
        case SystemState::ERROR:
            // Аварийная индикация
            digitalWrite(LED_PIN, millis() % 200 < 100);
            break;
            
        case SystemState::RECOVERY:
            systemRecovery();
            break;
            
        default:
            break;
    }
    
    // Логирование изменений состояния
    if (systemState != previousState) {
        Serial.printf("Смена состояния: %d -> %d\n", 
                     static_cast<int>(previousState), 
                     static_cast<int>(systemState));
        previousState = systemState;
              //  lastStateChange = millis();
    }
}

void processSensors() {
    static uint32_t lastIMURead = 0;
    
    if (millis() - lastIMURead >= ICM_READ_INTERVAL) {
        if (ICMHandler::readData()) {
            SensorData data = ICMHandler::getData();
            EleronController::update(data);
        }
        lastIMURead = millis();
    }
}

void handleRadioCommunication() {
    static uint32_t lastRadioProcess = 0;
    
    if (millis() - lastRadioProcess >= RADIO_PROCESS_INTERVAL) {
        radioReceiver.process(); // Будет обрабатывать состояние автомата
        //  radioReceiver.process();
        lastRadioProcess = millis();
    }
}

void controlOutputs() {
    // Управление через EleronController
}

void logTelemetry() {
    static uint32_t lastSDWrite = 0;
    
    if (millis() - lastSDWrite >= SD_WRITE_INTERVAL) {
        SensorData data = ICMHandler::getData();
        SDHandler::writeData(data);
        lastSDWrite = millis();
    }
}

void updateSystemStatus() {
    static uint32_t lastStatusUpdate = 0;
    
    if (millis() - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
        printStatusReport();
        lastStatusUpdate = millis();
    }
}

void handleEmergency() {
    Serial.println("!!! АВАРИЙНЫЙ РЕЖИМ !!!");
    EleronController::setNeutralPosition();
    
    static uint32_t emergencyStart = 0;
    if (emergencyStart == 0) {
        emergencyStart = millis();
    }
    
    if (millis() - emergencyStart > 10000) {
        systemState = SystemState::RECOVERY;
        emergencyStart = 0;
    }
}

void systemRecovery() {
    Serial.println("Попытка восстановления...");
    
    SDHandler::end();
    delay(1000);
    
    if (ICMHandler::begin() && radioReceiver.init() == EbyteStatus::SUCCESS) {
        systemState = SystemState::STANDBY;
        Serial.println("Восстановление успешно!");
    } else {
        Serial.println("Восстановление не удалось!");
        systemState = SystemState::ERROR;
    }
}

void printStatusReport() {
    Serial.println("\n=== СТАТУС СИСТЕМЫ ===");
    Serial.printf("Время работы: %lu сек\n", millis() / 1000);
    Serial.printf("Состояние: %d\n", static_cast<int>(systemState));
    Serial.printf("Ошибок: %lu\n", errorCount);
    Serial.printf("Память: %lu байт\n", ESP.getFreeHeap());
    Serial.printf("Чтения IMU: %lu\n", ICMHandler::getReadCount());
    Serial.printf("Radio State: %d\n", static_cast<int>(radioReceiver.getInternalState()));

    Serial.println("======================");
}

void checkSystemHealth() {
    static uint32_t lastHealthCheck = 0;
    
    if (millis() - lastHealthCheck >= ERROR_CHECK_INTERVAL) {
        // Проверка перегрева
        float temp = temperatureRead();
        if (temp > 75.0) {
            Serial.println("Перегрев процессора!");
            systemState = SystemState::ERROR;
        }
        
        // Проверка памяти
        if (ESP.getFreeHeap() < 10000) {
            Serial.println("Мало свободной памяти!");
        }
        
        lastHealthCheck = millis();
    }
}


