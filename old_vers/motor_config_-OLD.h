//  Отлично! Готовлю полную исправленную версию с улучшениями.
//  📁 Структура проекта:
//  text
//  
//  motor_controller/
//  ├── motor_controller.h          // Основной заголовочный файл
//  ├── motor_controller.cpp        // Реализация
//  ├── motor_config.h              // Конфигурация (новый файл)
//  └── motor_controller_private.h  // Приватные методы (новый файл)
//  
//  📄 motor_config.h
//  cpp
//  
#pragma once

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// Конфигурация аппаратных пинов
struct MotorHardwareConfig {
    static constexpr int ESC_A_PIN = 25;
    static constexpr int ESC_B_PIN = 26;
};

// Параметры ESC (микросекунды)
struct ESCConfig {
    static constexpr int MIN_US = 1000;      // 0% скорости (стоп)
    static constexpr int MAX_US = 2000;      // 100% скорости
    static constexpr int NEUTRAL_US = 1500;  // Нейтральное положение
    static constexpr int FREQUENCY = 50;     // 50 Hz для ESC
};

// Параметры тестового прогона
struct TestConfig {
    static constexpr int SPEED_PERCENT = 70;  // Тестовая скорость (70%)
    static constexpr int DURATION_SEC = 240;  // Длительность теста (240 сек)
    static constexpr int REPORT_INTERVAL = 30;// Интервал отчетов (сек)
};

// Конфигурация MCPWM
struct MCPWMConfig {
    static constexpr int GROUP_ID = 0;
    static constexpr int CLOCK_HZ = 10 * 1000 * 1000; // 10MHz
    static constexpr int PERIOD_TICKS = 200000;       // 50Hz: 10MHz / 50Hz
    static constexpr int INTR_PRIORITY = 0;
};

#endif

