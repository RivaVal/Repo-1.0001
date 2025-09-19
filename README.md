
            // 📋  СТРУКТУРА ПРОЕКТА
            
            text
            /Ваш_Проект/
            ├── E49_Config.h
            ├── E49_Controller.h
            ├── E49_Controller.cpp
            ├── SPI_Manager.h          ← Исправленный
            ├── SPI_Manager.cpp        ← Исправленный  
            ├── ICM_Handler.h          ← Исправленный
            ├── ICM_Handler.cpp        ← Исправленный
            ├── SD_Handler.h
            ├── SD_Handler.cpp
            ├── Eleron_Controller.h
            -- motor_controller.cpp
            -- motor_controller.h
            └── main.ino


                           СХЕМА ПОДКЛЮЧЕНИЯ
                  +---------------------------+
                  |          ESP32            |		Modules
                  |                           |
                  | GPIO23 (VSPI_MOSI)  ------+-----> To ICM-20948 SDA
                  |                           |-----> To SD Module MOSI
                  |                           |
                  | GPIO19 (VSPI_MISO)  ------+-----> To ICM-20948 SDO/ADO
                  |                           |-----> To SD Module MISO
                  |                           |
                  | GPIO18 (VSPI_SCLK)  ------+-----> To ICM-20948 SCL
                  |                           |-----> To SD Module SCK
                  |                           |
                  | GPIO22 (Custom CS)  ------+-----> To ICM-20948 NCS
                  |                           |
                  | GPIO33 (Custom CS)  ------+-----> To SD Module CS
                  +---------------------------+
                  | GPIO16  (Pin -)-----------+-----> To E49 Module Rx
                  | GPIO17  (Pin -)-----------+-----> To E49 Module Tx
                  | GPIO4   (Pin -) ----------+-----> To E49 Module M0
                  | GPIO21  (Pin -)-----------+-----> To E49 Module M1
                  | GPIO5   (Pin -)-----------+-----> To E49 Module AUX
                  +---------------------------+
              25    | GPIO32  (Cuatom )---------+-----> To Motor_1 Control
              26    | GPIO33  (Cuatom )---------+-----> To Motor_2 Control
                  |                           |
                  | Gnd	----------------------+-----> To Motor_2 Gnd // Заземлить оба канала
                  | 			                    |-----> To Motor_2 Gnd // Заземлить оба канала
                  +---------------------------+
                  | GPIO12  (Castom PWM)------+-----> To Servo_1 Control
                  | GPIO13  (Castom PWM)------+-----> To Servo_2 Control
                  | GPIO14  (Castom PWM)------+-----> To Servo_3 Control
                  | GPIO27  (Castom PWM)------+-----> To Servo_4 Control
                  | GPIO26  (Castom PWM)------+-----> To Servo_5 Control
                  |---------------------------+

Анализ занятых пинов:
Уже используются:
    GPIO16 - E49_RX
    GPIO17 - E49_TX
    GPIO4 - E49_M0
    GPIO5 - E49_AUX
    GPIO21 - E49_M1 ✅ (закреплен)
    GPIO18 - VSPI_SCLK
    GPIO19 - VSPI_MISO
    GPIO23 - VSPI_MOSI
    GPIO22 - ICM_CS
    GPIO25 - Motor_1
    GPIO26 - MOTOR_2
    GPIO32 - ICM_INT
    GPIO33 - SD_CS
    GPIO12-14, 26-27 - сервоприводы

4. Полная конфигурация:

// E49_Config.h - ПРАВИЛЬНЫЕ пины:
const uint8_t E49_PIN_RX = 16;
const uint8_t E49_PIN_TX = 17; 
const uint8_t E49_PIN_M0 = 4;
const uint8_t E49_PIN_M1 = 21;  // ✅ Правильно - GPIO 21
const uint8_t E49_PIN_AUX = 5;

// SPI пины - СВОБОДНЫ от конфликтов:
#define VSPI_SCLK 18   // Только для SPI!
#define VSPI_MISO 19   // Только для SPI!  
#define VSPI_MOSI 23   // Только для SPI!
#define ICM_CS    22   // Любой свободны

//-----------------------------------------------------

