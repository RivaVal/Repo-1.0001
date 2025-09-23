

//=============================================================================
//==============    ( motor_controller.h )  ===================================
//=============================================================================




//=============================================================================
//=============================================================================
//=============================================================================
//  ПРИВЕТ!! Окружение: Arduino IDE, микроконтроллер esp32, моторы без драйверов, 
//  управляются напрямую от ESP32 , используются два пина для двух моторов: GPIO25 для 
//  мотора_А и GPIO26 для мотора_Б ,моторы питаются через регулятор скорости BLHeli 
//  EMAX ESC от источника питания: 21V, 6500 mAh    Прошу  проанализировать код 
//  проекта из github, прошу на основе уже существующих мождулей, дать вариант МОДУЛЯ 
//  управления двумя моторами при движении моторов в одну сторону: QX Motor QF(2827) 2227-1800KV 
//  без драйвера, на основе последней актуальной версии mcpwm, НУЖЕН вариант кода, который 
//  можно вписать в существующню структуру проекта! Прошу дать вариант кода с полной 
//  документацией и подробным описанием, а так же добавить код для тестового прогона 
//  двигателей:
//  1.Последовательность работы при тестовом прогоне:
//  2. Плавный старт моторов до тестового уровня
//  3. Работа на тестовом уровне в течение 4 минут
//  4. Плавная остановка моторов !
//  
//  Привет! Проанализировал ваш проект. Вот модуль для управления моторами через 
//  ESC с тестовым прогоном:
//  Модуль управления моторами через ESC (motor_controller.h)
//  
//=============================================================================
//=============================================================================
//=============================================================================
//  / * *
//   * Модуль для управления двумя моторами QX Motor QF(2827) 2227-1800KV
//   * через ESC регуляторы BLHeli EMAX с использованием нового MCPWM Prelude API.
//   * Только движение в одну сторону.
//   * Версия: 2.0.0 (с актуальным MCPWM Prelude)
//   * 
//   * Подключение: 
//   * - GPIO25 -> ESC мотора A
//   * - GPIO26 -> ESC мотора B  
//   * Питание: 21V, 6500mAh
//  
//   * /
//   *   ( motor_controller.h )
//   *
//   *
//   * Только движение в одну сторону.
//   * Версия: 2.0.1 (исправлены предупреждения инициализации)
//   * /
//==============================================================
//	📄 motor_controller.h (исправленный)
//==============================================================
//  📄 motor_controller.h (единый файл)
//==============================================================
//📄 motor_controller.h (исправленная версия)
//==============================================================  

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <driver/mcpwm_prelude.h>

class MotorController {
private:
    // Конфигурация пинов ESC
    static const uint8_t ESC_A_PIN = 25;
    static const uint8_t ESC_B_PIN = 26;
    
    // Параметры ESC
    static const int ESC_MIN_US = 1000;
    static const int ESC_MAX_US = 2000;
    static const int PWM_FREQUENCY = 50;
    
    // Параметры тестового прогона
    static const int TEST_SPEED_PERCENT = 70;
    static const int TEST_DURATION_SEC = 240;
    
    // MCPWM объекты
    mcpwm_timer_handle_t timer_a = nullptr;
    mcpwm_timer_handle_t timer_b = nullptr;
    mcpwm_oper_handle_t oper_a = nullptr;
    mcpwm_oper_handle_t oper_b = nullptr;
    mcpwm_cmpr_handle_t comparator_a = nullptr;
    mcpwm_cmpr_handle_t comparator_b = nullptr;
    mcpwm_gen_handle_t generator_a = nullptr;
    mcpwm_gen_handle_t generator_b = nullptr;
    
    // Текущие скорости моторов
    int current_speed_a;
    int current_speed_b;
    
    // Флаги состояния
    bool is_test_running;
    unsigned long test_start_time;
    unsigned long last_report_time;
    
    // Приватные методы
    void init_mcpwm_components();
    void setup_generator_actions();
    void safe_delete(mcpwm_timer_handle_t& handle);
    void safe_delete(mcpwm_oper_handle_t& handle);
    void safe_delete(mcpwm_cmpr_handle_t& handle);
    void safe_delete(mcpwm_gen_handle_t& handle);
    
    int percent_to_us(int speed_percent) const;
    uint32_t us_to_ticks(int us) const;
    bool set_motor_speed(char motor_id, int speed_percent);
    void ramp_speed(int target_speed, float duration);

public:
    MotorController();
    ~MotorController();
    
    bool init_esc();
    bool set_motors_speed(int speed, float ramp_time = 0);
    void stop_motors(float ramp_time = 0);
    bool start_test_sequence();
    void stop_test_sequence();
    String get_motors_status() const;
    bool is_test_active() const;
    void update();
};

extern MotorController motor_controller;

#endif