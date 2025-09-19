



//  Интеграция с основным проектом (main.cpp)
//  

/**
 * Основной файл проекта с интеграцией управления моторами через ESC
 */

#include <Arduino.h>
#include "motor_controller.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // MotorController автоматически инициализируется
  Serial.println("Система управления моторами через ESC готова");
  
  // Опционально: ручной запуск теста
  // motor_controller.start_test_sequence();
}

void loop() {
  // Основная логика проекта
  motor_controller.update();
  
  // Интеграция с существующей логикой
  static unsigned long last_status_time = 0;
  if (millis() - last_status_time > 5000) {
    if (motor_controller.is_test_active()) {
      Serial.println(motor_controller.get_motors_status());
    }
    last_status_time = millis();
  }
  
  // Ваша существующая логика здесь...
  
  delay(100);
}

// Функции для внешнего управления
void set_motors_speed(int speed) {
  motor_controller.set_motors_speed(speed, 2.0);
}

void stop_motors() {
  motor_controller.stop_motors(2.0);
}

void start_motor_test() {
  motor_controller.start_test_sequence();
}


/*  
Схема подключения

ESP32 GPIO25 ──── Signal ── ESC A ── Motor A
ESP32 GPIO26 ──── Signal ── ESC B ── Motor B
ESP32 GND ─────── GND ───── ESC A/B
21V Power ─────── Power ─── ESC A/B

Важные замечания:

    Калибровка ESC: Некоторые ESC требуют калибровки перед первым использованием
    Безопасность: Всегда проверяйте подключение перед подачей питания 21V
    Охлаждение: Обеспечьте хорошее охлаждение ESC во время теста
    Последовательность запуска: Сначала питание ESP32, потом питание ESC
Последовательность тестового прогона:
    ✅ Плавный старт от 0% до 70% за 5 секунд
    ✅ Работа на 70% скорости в течение 4 минут
    ✅ Плавная остановка от 70% до 0% за 4 секунды
    ✅ Мониторинг и отчетность каждые 30 секунд
Модуль готов к интеграции в ваш проект!
*/




