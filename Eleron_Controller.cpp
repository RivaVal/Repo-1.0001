

      //===============================================
      //  4. Исправленный Eleron_Controller.cpp
      //=================================================

      //  #include "Eleron_Controller.h"
      //  #include <driver/ledc.h>
        //      Eleron_Controller.cpp
        //  cpp
#include "Eleron_Controller.h"
#include <driver/ledc.h>
#include <math.h>

        //          #define PI 3.14159265358979323846

// Конфигурация сервоприводов
const int servoPins[] = {12, 13, 14, 27, 26};
const ledc_channel_t channels[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, 
    LEDC_CHANNEL_3, LEDC_CHANNEL_4
};

// Статические переменные
uint32_t EleronController::last_update_time = 0;
int EleronController::current_angles[5] = {90, 90, 90, 90, 90};
bool EleronController::test_running = false;
uint32_t EleronController::test_start_time = 0;

void EleronController::begin() {
    // Настройка таймера LEDC
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_12_BIT;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);
    
    // Настройка каналов
    for (int i = 0; i < 5; i++) {
        ledc_channel_config_t channel_conf = {};
        channel_conf.gpio_num = servoPins[i];
        channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_conf.channel = channels[i];
        channel_conf.intr_type = LEDC_INTR_DISABLE;
        channel_conf.timer_sel = LEDC_TIMER_0;
        channel_conf.duty = 0;
        channel_conf.hpoint = 0;
        ledc_channel_config(&channel_conf);
        
        setServoAngle(i, 90); // Нейтральное положение
    }
    
    last_update_time = millis();
    Serial.println("✅ EleronController initialized");
}

void EleronController::setNeutralPosition() {
    for (int i = 0; i < 5; i++) {
        setServoAngle(i, 90); // Установка нейтрального положения
    }
    Serial.println("✅ Servos set to neutral position");
}

void EleronController::update(const SensorData& data) {
    uint32_t current_time = millis();
    
    // Защита от переполнения millis()
    if ((current_time - last_update_time) < UPDATE_INTERVAL && 
        current_time >= last_update_time) {
        return;
    }
    
    if (test_running) {
        runTestSequence(); // Запуск тестового прогона
    } else {
        processIMUData(data); // Нормальная работа
    }
    
    last_update_time = current_time;
}

void EleronController::processIMUData(const SensorData& data) {
    // Ваша логика обработки данных
    int pitch_angle = map(data.euler[1], -90, 90, 0, 180);
    int roll_angle = map(data.euler[0], -90, 90, 0, 180);
    
    setServoAngle(0, roll_angle);
    setServoAngle(1, pitch_angle);
    setServoAngle(2, 90);
    setServoAngle(3, 180 - roll_angle);
    setServoAngle(4, 90);
}

void EleronController::setServoAngle(uint8_t channel, int angle) {
    if (channel >= 5) return; // Защита от выхода за границы
    
    angle = constrain(angle, 0, 180);
    current_angles[channel] = angle;
    
    // Преобразование угла в микросекунды (500-2400μs)
    int pulse_width = map(angle, 0, 180, 500, 2400);
    
    // Преобразование в duty цикл (12-bit resolution, 50Hz)
    uint32_t duty = (pulse_width * 4096UL) / 20000UL;
    
    // Защита от переполнения (вместо constrain для unsigned)
    if (duty > 4095) duty = 4095;
    
    ledcWrite(channels[channel], duty);
}

void EleronController::startTestSequence() {
    test_running = true;
    test_start_time = millis();
    Serial.println("🚀 Starting servo test sequence for all 5 servos...");
}

void EleronController::stopTestSequence() {
    test_running = false;
    Serial.println("🛑 Servo test sequence stopped");
    setNeutralPosition(); // Возврат в нейтральное положение
}

void EleronController::runTestSequence() {
    if (!test_running) return;
    
    uint32_t current_time = millis();
    uint32_t elapsed = current_time - test_start_time;
    uint32_t phase_time = elapsed % 8000; // 8-секундный цикл
    
    // Тестовые паттерны для каждого сервопривода
    for (int i = 0; i < 5; i++) {
        int angle = 90; // По умолчанию нейтральное положение
        
        // Разные паттерны для каждого сервопривода
        switch (i) {
            case 0: // Серво 0: плавное движение 0-180-0
                angle = (phase_time < 4000) ? 
                    map(phase_time, 0, 4000, 0, 180) :
                    map(phase_time, 4000, 8000, 180, 0);
                break;
                
            case 1: // Серво 1: быстрые колебания
                angle = 90 + 60 * sin(2 * PI * phase_time / 2000.0);
                break;
                
            case 2: // Серво 2: ступенчатое движение
                if (phase_time < 2000) angle = 45;
                else if (phase_time < 4000) angle = 90;
                else if (phase_time < 6000) angle = 135;
                else angle = 90;
                break;
                
            case 3: // Серво 3: медленное движение
                angle = map(phase_time, 0, 8000, 0, 180);
                break;
                
            case 4: // Серво 4: случайные движения
                angle = 90 + 30 * sin(2 * PI * phase_time / 3000.0) + 
                        20 * cos(2 * PI * phase_time / 5000.0);
                break;
        }
        
        angle = constrain(angle, 0, 180);
        setServoAngle(i, angle);
    }
    
    // Периодический отчет
    if (elapsed % 1000 < 20) {
        Serial.printf("🔧 Test running: %lu ms | Angles: %d, %d, %d, %d, %d\n",
            elapsed, current_angles[0], current_angles[1], current_angles[2],
            current_angles[3], current_angles[4]);
    }
    
    // Автоматическая остановка через 60 секунд
    if (elapsed > 60000) {
        stopTestSequence();
    }
}

