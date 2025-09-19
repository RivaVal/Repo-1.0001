


    //=========================================================
    //  Eleron_Controller.h 
    //===========================================================
    //  📋 Полная исправленная версия:
    //  Eleron_Controller.h
    //  cpp

#pragma once
#include <Arduino.h>
#include "E49_Config.h"
        //  #include "servo_config.h"

class EleronController {
private:
    static const uint32_t UPDATE_INTERVAL = 20;
    static uint32_t last_update_time;
    static int current_angles[5];
    static bool test_running;
    static uint32_t test_start_time;
    static uint8_t test_phase;

public:
    static void begin();
    static void update(const SensorData& data);
    static const int* getCurrentAngles() { return current_angles; }
    
    static void setNeutralPosition();
    static void startTestSequence();
    static void stopTestSequence();
    static bool isTestRunning() { return test_running; }

private:
    static void processIMUData(const SensorData& data);
    static void setServoAngle(uint8_t channel, int angle);
    static void runTestSequence();
};
