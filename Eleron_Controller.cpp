
      //
      //  4. Исправленный Eleron_Controller.cpp
      //  cpp

#include "Eleron_Controller.h"
#include <driver/ledc.h>
            //  #include <ledc.h>  // Добавьте для ledcWrite
            //#include "Eleron_Controller.h"

            // Добавьте определение массива servoPins
            //const int servoPins[] = {12, 13, 14, 27, 26};

// Оставьте определения здесь:
const int servoPins[] = {12, 13, 14, 27, 26};
const ledc_channel_t channels[] = {
    LEDC_CHANNEL_0, 
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, 
    LEDC_CHANNEL_4
};


// Определение статических переменных
uint32_t EleronController::last_update_time = 0;
int EleronController::current_angles[5] = {90, 90, 90, 90, 90};

      //  uint32_t EleronController::last_update_time = 0;
      //  int EleronController::current_angles[5] = {0};

void EleronController::begin() {
    // Инициализация сервоприводов
    for (int i = 0; i < 5; i++) {
        pinMode(servoPins[i], OUTPUT);
        current_angles[i] = 90;
    }
    last_update_time = millis();
}

void EleronController::update(const SensorData& data) {
    if (millis() - last_update_time < UPDATE_INTERVAL) {
        return;
    }
    processIMUData(data);
    last_update_time = millis();
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
    angle = constrain(angle, 0, 180);
    current_angles[channel] = angle;
    
    int pulse_width = map(angle, 0, 180, 500, 2400);
    // Используйте pulse_width
    ledcWrite(channels[channel], pulse_width);  // channels должен быть определен в E49_Config.h
}
