


    //=========================================================
    //  Eleron_Controller.h 
    //===========================================================

//
#pragma once
#include <Arduino.h>
#include "E49_Config.h"  // Добавьте этот include


/*  
const int servoPins[] = {12, 13, 14, 27, 26};
const ledc_channel_t channels[] = {
    LEDC_CHANNEL_0, 
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, 
    LEDC_CHANNEL_4
};
*/

class EleronController {
private:
    static const uint32_t UPDATE_INTERVAL = 20;
    static uint32_t last_update_time;
    static int current_angles[5];  // Добавьте это объявление

public:
    static void begin();
    static void update(const SensorData& data);
    static const int* getCurrentAngles() { return current_angles; }
    static void setNeutralPosition() {
        // Установка сервоприводов в нейтральное положение
        for (int i = 0; i < 5; i++) {
            setServoAngle(i, 90); // 90 градусов - нейтральное положение
        }
    }


private:
    static void processIMUData(const SensorData& data);
    static void setServoAngle(uint8_t channel, int angle);  // Сделайте методом класса
};
