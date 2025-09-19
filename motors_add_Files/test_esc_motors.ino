



//  Тестовый скрипт (test_esc_motors.ino)
//  

/**
 * Тестовый скрипт для проверки моторов через ESC
 * Последовательность тестового прогона:
 * 1. Плавный старт до 70% (5 сек)
 * 2. Работа на 70% в течение 4 минут
 * 3. Плавная остановка (4 сек)
 */

#include "motor_controller.h"

// Переменные для управления тестом
bool test_completed = false;
unsigned long last_display_time = 0;

void setup() {
  Serial.begin(115200);
  delay(3000); // Даем время для инициализации ESC
  
  Serial.println("=== СИСТЕМА УПРАВЛЕНИЯ МОТОРАМИ ЧЕРЕЗ ESC ===");
  Serial.println("Модель: QX Motor QF(2827) 2227-1800KV");
  Serial.println("Регулятор: BLHeli EMAX ESC");
  Serial.println("Питание: 21V 6500mAh");
  Serial.println("ESC пины: GPIO25 (Мотор A), GPIO26 (Мотор B)");
  Serial.println("ШИМ: 50Hz, 1000-2000μs");
  Serial.println("=============================================");
  
  // Даем дополнительные инструкции для ESC
  Serial.println("Перед запуском убедитесь:");
  Serial.println("1. ESC подключены к питанию 21V");
  Serial.println("2. Моторы правильно подключены к ESC");
  Serial.println("3. Сигнальные провода подключены к GPIO25/26");
  Serial.println("4. GND ESP32 соединен с GND питания ESC");
  
  delay(2000);
  
  // Автозапуск тестового прогона
  start_test_sequence();
}

void loop() {
  // Обновление состояния моторов
  motor_controller.update();
  
  // Отображение статуса каждые 10 секунд
  if (millis() - last_display_time > 10000) {
    display_status();
    last_display_time = millis();
  }
  
  // Перезапуск теста после завершения
  if (!motor_controller.is_test_active() && !test_completed) {
    test_completed = true;
    Serial.println("\nТест завершен! Команды:");
    Serial.println("R - Перезапуск теста");
    Serial.println("S - Экстренная остановка");
    Serial.println("0-9 - Ручное управление скоростью");
    Serial.println("? - Помощь");
  }
  
  // Обработка команд с Serial
  if (Serial.available()) {
    handle_serial_command();
  }
  
  delay(100);
}

void start_test_sequence() {
  test_completed = false;
  Serial.println("\nИнициализация ESC...");
  delay(1000);
  
  Serial.println("Запуск тестового прогона...");
  motor_controller.start_test_sequence();
}

void display_status() {
  if (motor_controller.is_test_active()) {
    String status = motor_controller.get_motors_status();
    Serial.print("Статус: ");
    Serial.println(status);
  }
}

void handle_serial_command() {
  char command = Serial.read();
  
  switch (command) {
    case 'r':
    case 'R':
      if (!motor_controller.is_test_active()) {
        start_test_sequence();
      }
      break;
      
    case 's':
    case 'S':
      if (motor_controller.is_test_active()) {
        Serial.println("Принудительная остановка теста...");
        motor_controller.stop_test_sequence();
      }
      break;
      
    case '0':
      motor_controller.set_motors_speed(0, 2.0); // Стоп
      break;
    case '1':
      motor_controller.set_motors_speed(10, 2.0);
      break;
    case '2':
      motor_controller.set_motors_speed(20, 2.0);
      break;
    case '3':
      motor_controller.set_motors_speed(30, 2.0);
      break;
    case '4':
      motor_controller.set_motors_speed(40, 2.0);
      break;
    case '5':
      motor_controller.set_motors_speed(50, 2.0);
      break;
    case '6':
      motor_controller.set_motors_speed(60, 2.0);
      break;
    case '7':
      motor_controller.set_motors_speed(70, 2.0);
      break;
    case '8':
      motor_controller.set_motors_speed(80, 2.0);
      break;
    case '9':
      motor_controller.set_motors_speed(90, 2.0);
      break;
      
    case '?':
      print_help();
      break;
  }
}

void print_help() {
  Serial.println("\n=== КОМАНДЫ УПРАВЛЕНИЯ ===");
  Serial.println("R - Запуск тестового прогона (4 мин)");
  Serial.println("S - Экстренная остановка");
  Serial.println("0 - Стоп (0%)");
  Serial.println("1-9 - Скорость 10-90%");
  Serial.println("? - Эта справка");
  Serial.println("==========================");
}

