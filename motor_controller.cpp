


//  Реализация модуля (motor_controller.cpp)
//
//  Обновленная реализация (motor_controller.cpp)
//
//
//  Исправленная реализация (motor_controller.cpp)
//  

#include "motor_controller.h"
#include <Arduino.h>

// Глобальный экземпляр
MotorController motor_controller;

MotorController::MotorController() 
  : current_speed_a(0), current_speed_b(0), 
    is_test_running(false), test_start_time(0) {
  
  // Инициализация MCPWM для ESC с новым API
  init_esc_mcpwm_prelude();
  
  Serial.println("MotorController инициализирован (MCPWM Prelude)");
  Serial.println("ESC пины: GPIO25 (Мотор A), GPIO26 (Мотор B)");
  Serial.println("Частота ШИМ: 50Hz для ESC");
  Serial.println("Диапазон импульсов: 1000-2000μs");
}

MotorController::~MotorController() {
  // Очистка ресурсов MCPWM
  if (generator_a) mcpwm_del_generator(generator_a);
  if (generator_b) mcpwm_del_generator(generator_b);
  if (comparator_a) mcpwm_del_comparator(comparator_a);
  if (comparator_b) mcpwm_del_comparator(comparator_b);
  if (oper_a) mcpwm_del_operator(oper_a);
  if (oper_b) mcpwm_del_operator(oper_b);
  if (timer_a) mcpwm_del_timer(timer_a);
  if (timer_b) mcpwm_del_timer(timer_b);
}

void MotorController::init_esc_mcpwm_prelude() {
  Serial.println("Инициализация MCPWM Prelude API");
  
  // Конфигурация таймеров с полной инициализацией
  mcpwm_timer_config_t timer_config = {};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = 10 * 1000 * 1000; // 10MHz
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = 200000; // 50Hz: 10MHz / 50Hz = 200000 ticks
  timer_config.intr_priority = 0; // Добавлено
  timer_config.flags.update_period_on_empty = true;
  timer_config.flags.update_period_on_sync = true;
  timer_config.flags.allow_pd = false; // Добавлено
  
  // Создание таймеров
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_a));
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_b));
  
  // Конфигурация операторов с полной инициализацией
  mcpwm_operator_config_t operator_config = {};
  operator_config.group_id = 0;
  operator_config.intr_priority = 0; // Добавлено
  operator_config.flags.update_gen_action_on_tez = true;
  operator_config.flags.update_gen_action_on_tep = false;
  operator_config.flags.update_gen_action_on_sync = false;
  operator_config.flags.update_dead_time_on_tez = false;
  operator_config.flags.update_dead_time_on_tep = false;
  operator_config.flags.update_dead_time_on_sync = false;
  
  // Создание операторов
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_a));
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_b));
  
  // Привязка операторов к таймерам
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_a, timer_a));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_b, timer_b));
  
  // Конфигурация компараторов с полной инициализацией
  mcpwm_comparator_config_t comparator_config = {};
  comparator_config.intr_priority = 0; // Добавлено
  comparator_config.flags.update_cmp_on_tez = true;
  comparator_config.flags.update_cmp_on_tep = false;
  comparator_config.flags.update_cmp_on_sync = false;
  
  // Создание компараторов
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper_a, &comparator_config, &comparator_a));
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper_b, &comparator_config, &comparator_b));
  
  // Конфигурация генераторов
  mcpwm_generator_config_t generator_config = {};
  
  // Генератор для мотора A (GPIO25)
  generator_config.gen_gpio_num = ESC_A_PIN;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper_a, &generator_config, &generator_a));
  
  // Генератор для мотора B (GPIO26)
  generator_config.gen_gpio_num = ESC_B_PIN;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper_b, &generator_config, &generator_b));
  
  // Настройка действий генераторов для мотора A
  ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
    generator_a,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
    MCPWM_GEN_TIMER_EVENT_ACTION_END()
  ));
  
  ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
    generator_a,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a, MCPWM_GEN_ACTION_LOW),
    MCPWM_GEN_COMPARE_EVENT_ACTION_END()
  ));
  
  // Настройка действий генераторов для мотора B
  ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
    generator_b,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
    MCPWM_GEN_TIMER_EVENT_ACTION_END()
  ));
  
  ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
    generator_b,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b, MCPWM_GEN_ACTION_LOW),
    MCPWM_GEN_COMPARE_EVENT_ACTION_END()
  ));
  
  // Запуск таймеров
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer_a));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_a, MCPWM_TIMER_START_NO_STOP));
  
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer_b));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_b, MCPWM_TIMER_START_NO_STOP));
  
  // Установка начальной скорости 0
  set_motors_speed(0);
}

void MotorController::init_esc() {
  // Уже инициализировано в конструкторе
  Serial.println("ESC инициализированы с MCPWM Prelude API");
}

int MotorController::percent_to_us(int speed_percent) {
  speed_percent = constrain(speed_percent, 0, 100);
  return ESC_MIN_US + (speed_percent * (ESC_MAX_US - ESC_MIN_US) / 100);
}

void MotorController::set_esc_speed(int speed_percent, char motor_id) {
  int pulse_width_us = percent_to_us(speed_percent);
  
  // Преобразование микросекунд в тики таймера (10MHz clock)
  // Период для 50Hz: 10,000,000 / 50 = 200,000 ticks
  // Импульс: (pulse_width_us * 10,000,000) / 1,000,000 = pulse_width_us * 10
  uint32_t pulse_ticks = (pulse_width_us * 200000) / 20000; // Упрощенный расчет
  
  // Установка значения компаратора
  if (motor_id == 'A') {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, pulse_ticks));
    current_speed_a = speed_percent;
  } else {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, pulse_ticks));
    current_speed_b = speed_percent;
  }
}

void MotorController::ramp_speed(int target_speed, float duration) {
  int start_speed = current_speed_a;
  int steps = duration * 10; // 10 шагов в секунду
  float step_size = (target_speed - start_speed) / steps;
  
  for (int i = 0; i < steps; i++) {
    int intermediate_speed = start_speed + step_size * (i + 1);
    intermediate_speed = constrain(intermediate_speed, 0, 100);
    
    set_esc_speed(intermediate_speed, 'A');
    set_esc_speed(intermediate_speed, 'B');
    
    delay(100);
  }
}

bool MotorController::set_motors_speed(int speed, float ramp_time) {
  if (speed < 0 || speed > 100) {
    Serial.println("Ошибка: Скорость должна быть от 0 до 100%");
    return false;
  }
  
  if (ramp_time > 0) {
    ramp_speed(speed, ramp_time);
  } else {
    set_esc_speed(speed, 'A');
    set_esc_speed(speed, 'B');
  }
  
  int pulse_us = percent_to_us(speed);
  Serial.printf("Моторы: %d%% (%dμs)\n", speed, pulse_us);
  return true;
}

void MotorController::stop_motors(float ramp_time) {
  if (ramp_time > 0) {
    ramp_speed(0, ramp_time);
  } else {
    set_esc_speed(0, 'A');
    set_esc_speed(0, 'B');
  }
  Serial.println("Моторы остановлены");
}

void MotorController::start_test_sequence() {
  if (is_test_running) {
    Serial.println("Тест уже выполняется");
    return;
  }
  
  Serial.println("=== ЗАПУСК ТЕСТОВОГО ПРОГОНА ESC ===");
  Serial.println("1. Плавный старт до 70%...");
  
  is_test_running = true;
  test_start_time = millis();
  
  set_motors_speed(TEST_SPEED_PERCENT, 5.0);
  
  Serial.println("2. Работа на тестовом уровне (4 минуты)...");
  Serial.printf("Скорость: %d%%, Импульс: %dμs\n", TEST_SPEED_PERCENT, percent_to_us(TEST_SPEED_PERCENT));
}

void MotorController::stop_test_sequence() {
  if (!is_test_running) return;
  
  Serial.println("3. Плавная остановка моторов...");
  stop_motors(4.0);
  
  is_test_running = false;
  unsigned long test_duration = (millis() - test_start_time) / 1000;
  Serial.printf("Тест завершен! Длительность: %lu секунд\n", test_duration);
}

String MotorController::get_motors_status() {
  unsigned long elapsed_seconds = is_test_running ? (millis() - test_start_time) / 1000 : 0;
  
  char buffer[150];
  snprintf(buffer, sizeof(buffer), 
           "{\"motor_a\":%d,\"motor_b\":%d,\"pulse_us\":%d,\"test_running\":%s,\"test_time\":%lu,\"test_total\":%d}",
           current_speed_a, current_speed_b, percent_to_us(current_speed_a),
           is_test_running ? "true" : "false", elapsed_seconds, TEST_DURATION_SEC);
  return String(buffer);
}

bool MotorController::is_test_active() {
  return is_test_running;
}

void MotorController::update() {
  if (!is_test_running) return;
  
  unsigned long current_time = millis();
  unsigned long elapsed_seconds = (current_time - test_start_time) / 1000;
  
  static unsigned long last_report = 0;
  if (elapsed_seconds - last_report >= 30) {
    Serial.printf("Тест: %lu/%d сек\n", elapsed_seconds, TEST_DURATION_SEC);
    last_report = elapsed_seconds;
  }
  
  if (elapsed_seconds >= TEST_DURATION_SEC) {
    stop_test_sequence();
  }
}


/*

Упрощенная версия с обычным PWM (альтернатива)

Если с MCPWM Prelude будут проблемы, вот версия с обычным PWM:
cpp

/ * *
 * Альтернативная версия с использованием стандартного PWM
 * /

#include <Arduino.h>

class MotorController {
private:
  const int ESC_A_PIN = 25;
  const int ESC_B_PIN = 26;
  const int PWM_FREQ = 50;
  const int PWM_RESOLUTION = 16;
  
  int percent_to_duty(int speed_percent) {
    return map(speed_percent, 0, 100, 3275, 6553); // Для 16-bit resolution
  }
  
public:
  MotorController() {
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ESC_A_PIN, 0);
    ledcAttachPin(ESC_B_PIN, 1);
    set_motors_speed(0);
  }
  
  void set_motors_speed(int speed) {
    int duty = percent_to_duty(speed);
    ledcWrite(0, duty);
    ledcWrite(1, duty);
  }
};

//  Ключевые изменения:
        Ключевые изменения:
    ✅ Используется #include <driver/mcpwm_prelude.h>
    ✅ Новый API MCPWM с handles вместо legacy функций
    ✅ Правильное управление ресурсами (создание/удаление)
    ✅ Современная конфигурация таймеров и генераторов
    ✅ Обработка ошибок через ESP_ERROR_CHECK
Теперь код использует актуальный драйвер MCPWM без deprecated предупреждений!
*/

//-------------------------------------------------------------------------

/*
Альтернативная упрощенная версия с LEDC PWM

Если MCPWM Prelude все равно вызывает проблемы, вот надежная версия с LEDC:
cpp

/ * *
 * Упрощенная версия с LEDC PWM для ESC
 * /

#include <Arduino.h>

class SimpleMotorController {
private:
  const int ESC_A_PIN = 25;
  const int ESC_B_PIN = 26;
  const int PWM_FREQ = 50;
  const int PWM_RESOLUTION = 16;
  
  int current_speed_a = 0;
  int current_speed_b = 0;
  bool is_test_running = false;
  unsigned long test_start_time = 0;
  
  int percent_to_duty(int speed_percent) {
    // Для 16-bit resolution: 0% = 3275, 100% = 6553
    return map(constrain(speed_percent, 0, 100), 0, 100, 3275, 6553);
  }
  
  void ramp_speed(int target_speed, float duration) {
    int start_speed = current_speed_a;
    int steps = duration * 10;
    float step_size = (target_speed - start_speed) / steps;
    
    for (int i = 0; i < steps; i++) {
      int speed = start_speed + step_size * (i + 1);
      set_motors_speed(speed, 0);
      delay(100);
    }
  }

public:
  SimpleMotorController() {
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(ESC_A_PIN, 0);
    ledcAttachPin(ESC_B_PIN, 1);
    set_motors_speed(0);
    Serial.println("SimpleMotorController инициализирован");
  }
  
  void set_motors_speed(int speed, float ramp_time = 0) {
    speed = constrain(speed, 0, 100);
    
    if (ramp_time > 0) {
      ramp_speed(speed, ramp_time);
    } else {
      int duty = percent_to_duty(speed);
      ledcWrite(0, duty);
      ledcWrite(1, duty);
      current_speed_a = speed;
      current_speed_b = speed;
      Serial.printf("Моторы: %d%%\n", speed);
    }
  }
  
  void start_test_sequence() {
    Serial.println("Запуск теста...");
    set_motors_speed(70, 5.0);
    is_test_running = true;
    test_start_time = millis();
  }
  
  void update() {
    if (is_test_running && millis() - test_start_time > 240000) {
      set_motors_speed(0, 4.0);
      is_test_running = false;
      Serial.println("Тест завершен");
    }
  }
};

SimpleMotorController motor_controller;

Ключевые исправления:
    ✅ Полная инициализация всех полей структур
    ✅ Добавлены intr_priority = 0 и allow_pd = false
    ✅ Исправлена конфигурация действий генераторов
    ✅ Упрощен расчет импульсов для ESC
    ✅ Добавлена альтернативная версия с LEDC
Теперь код должен компилироваться без предупреждений!
Теперь код использует актуальный драйвер MCPWM без deprecated предупреждений!



*/