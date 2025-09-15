

//НОВАЯ РЕАЛИЗАЦИЯ __Set_Full_0005_001   :: Переделанный проект под MODULEMANAGER
//
//Исправленный код (Receiver)
/**
 * RECEIVER MODULE for Ebyte E49 400T20D
 * Версия 1.0
 */


/**
 * @file Receiver_Controller.ino
 * @brief Основной контроллер приемника для управления самолетом
 * 
 * Особенности:
 * - Прием данных через модуль EBYTE (LoRa)
 * - Управление сервоприводами элеронов через LEDC
 * - Управление моторами через MCPWM
 * - Плавное перемещение сервоприводов
 * - Обработка команд с пульта управления
 * 
 * @author Your Name
 * @date 2025
 */

// Системные библиотеки
#include <Arduino.h>
#include <stdint.h>
                    //#include <cstdint.h>
#include <cstddef>

// Периферийные библиотеки ESP32
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include "esp_err.h"

// Библиотеки для работы модуля ICM_20948 и SD Card
#include <SPI.h>
#include <SD.h>
#include "ICM_20948.h"  // Библиотека для ICM-20948
#include <math.h>

// Пользовательские библиотеки
#include "EBYTE.h"
#include "E49_Config.h"
#include "motor_config.h"
#include "E49_Controller.h"




// Определение модулей
#define DEBUG_MODE
#define MODULE_ICM_20948
#define MODULE_SD_CARD 
  //  #define MODULE_EBYTE
  //  #define EBYTE_MODULE
  //  #define EBYTE_RADIO_MODULE
#define MODULE_LEDC_SERVO
  //  #define MODULE_MCPWM_MOTOR
#define CONTROL_PROJECT_MODULE
#define ESP32FULL 3238

// ==================================================================================
// КОНСТАНТЫ И ПЕРЕМЕННЫЕ
// ==================================================================================

// Константы для LEDC элеронов
#ifdef LEDC_ELERON_MODULE
#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES        LEDC_TIMER_8_BIT
#define LEDC_DUTY            (128)
#define LEDC_FREQUENCY       (50)
#define ELERON_UPDATE_DELAY_MS 150
#endif

        /*    
Почему это лучшее решение: VSPI ??
    Используем аппаратный SPI - максимальная производительность
    Минимальные изменения - только один пин EBYTE Заменяется
    Вместо GPIO18 - используем свободный GPIO21
    Стандартные библиотеки - полная совместимость
    Стабильная работа - аппаратный SPI надежнее software
        */

// Глобальные переменные
uint32_t last_success_time = 0;
uint32_t main_loop_counter = 0;
const uint8_t main_loop_counter_m_LIMIT = 150;
uint32_t eleron_test_count = 0;
const uint16_t eleron_max_pause = 150;
uint8_t eleron_count = 0;
uint8_t loop_curent_state = 0;


// Глобальные переменные
ICM_20948_SPI myICM;  // Создаем объект ICM-20948
File dataFile;         // Файл для записи данных
  //  SPIClass hspi(HSPI);   // Создаем объект для HSPI
SPIClass vspi(VSPI);   // Создаем объект для VSPI

// Структура настройки отладки
ControllerDebug_t _debug = {
    .RC_Debug          = false ,
    .recieve_Debug     = false ,  // обратите внимание на написание!
    //.receiver_init     = false ,  // Отладочный вывод функции init()
    .sender_Debug      = false ,
    .ebyte_Debug       = false ,
    .mcpwm_Debug       = false ,
    .ledc_Debug        = false ,
    .setup_Debug       = true ,
    .loop_Debug        = false ,
    .diod_Debug        = false ,
    .icm20984_Debug    = false ,
    .eleron_Debug      = false ,         // Debug ALL eleron prog
    .slidePot_Debug    = false ,      // Отладка для SLIDE_POT проекта
    .throttle_Debug    = false ,        // Debug ALL Throttle prog   
    .joystick_Debug    = false ,
    .incomedata_Debug  = false ,
    .processdata_Debug    = false ,    // Debug all process function
    .cc               = false 
 };

// Данные команд
DataComSet_t Com;

// Конфигурация сервоприводов
        // const int servoPins[] = {12, 13, 14, 27, 26};
const int servoPins[] = {12, 13, 14, 27, 26};
const ledc_channel_t channels[] = {
    LEDC_CHANNEL_0, 
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4
};

// Углы сервоприводов
int angles[5] = {0, 0, 0, 0, 0};
int targetAngles[5] = {0, 0, 0, 0, 0};

// Контроллеры EBYTE
E49_Controller sender(false);
E49_Controller receiver(true);

// Тестовые данные
DataComSet_t testData = {
    .preamble = {0xAA, 0x55},
    .packet_id = 0,
    .comUp = 89,
    .comLeft = 89,
    .comThrottle = 127,
    .comParashut = 0,
    .timestamp = 0,
    .comSetAll = 0b00000001,
    .crc8 = 0
};

// ==================================================================================
// ПРОТОТИПЫ ФУНКЦИЙ
// ==================================================================================

// EBYTE функции
void setup_ebyte();
void loop_ebyte();

// MCPWM функции (мотора)
#ifdef MCPWM_MOTOR_MODULE
void setup_mcpwm();
void loop_mcpwm();
void init_motors();
void run_test_sequence();
void smooth_start_motors();
void smooth_stop_motors();
void set_motor_power(uint8_t motor_num, uint8_t power);
#endif

// LEDC функции (элероны)
#ifdef LEDC_ELERON_MODULE
void setupLEDCTimer();
void setupLEDCChannel(int chan_i, int pin);
void Eleron_LEDCV2_setup();
void proceed_air_command();
void smoothUpdate();
void setServoAngle(int chan_i, int angle);
void Eleron_LEDCV2_loop();
#endif

// Вспомогательные функции
void prepare_income_data();
void prepare_Com_data();

// Прототипы функций ICM-20948
bool initSDCard();
bool initICM20948();
void calculateEulerAngles(float *quat, float *euler);
void writeBinaryData(const SensorData &data);
void blinkLED(int times, int delayTime);

// ================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==================
EBYTE Transceiver(&Serial2, E49_PIN_M0, E49_PIN_M1, E49_PIN_AUX);


// Дополнительные данные для класса moduleMahager
// В основном .ino файле добавьте:
        //  extern E49_Controller sender;
        //  extern E49_Controller receiver;
        //  extern ModuleTimers_t moduleTimers;
        //ModuleTimers_t moduleTimers;
// Обнулим все таймеры для начала!        
ModuleTimers_t moduleTimers = {0, 0, 0, 0, 0, 0, 0};

// И создайте объект после объявления контроллеров:
        //ModuleManager moduleManager ;
        //moduleManager(false, sender, receiver);

// Статистика
//  static uint32_t packets_received = 0;
//  static uint32_t packets_ack_sent = 0;
//  static uint32_t last_packet_time = 0;
//  static uint32_t error_count = 0;

// Данные
//  static DataComSet_t received_data;
//  static AckPacket_t ack_data;


      // Работа с модулем моторов
      // / ========================================================================================
      //    2. Основной файл qx_motor_control.ino
      //========================================================================================
    /**
    * @file qx_motor_control.ino
    * @brief Программа управления 2 моторами QX Motor QF(2827) 2227-1800KV
    * 
    * Особенности:
    * - Управление только в одну сторону
    * - Плавный старт и остановка
    * - Автоматический 4-минутный тест на 65% мощности
    * - Защитные ограничения мощности
    * - Подробный вывод в Serial монитор
    */
    // =============================================
    // ===      Дескрипторы  MCPWM        ===
    // =============================================

    static mcpwm_cmpr_handle_t comparator_a = NULL; ///< Компаратор для мотора A
    static mcpwm_cmpr_handle_t comparator_b = NULL; ///< Компаратор для мотора B
    static mcpwm_gen_handle_t generator_a = NULL;   ///< Генератор для мотора A
    static mcpwm_gen_handle_t generator_b = NULL;   ///< Генератор для мотора B
    static mcpwm_timer_handle_t timer_a = NULL;     ///< Таймер для мотора A
    static mcpwm_timer_handle_t timer_b = NULL;     ///< Таймер для мотора B

    // =============================================
    // ===    Основные функции      ===
    // =============================================
    // ==================================================================================
    // РЕАЛИЗАЦИЯ ФУНКЦИЙ
    // ==================================================================================
    // ================== РЕАЛИЗАЦИЯ ФУНКЦИЙ ===================================
    // Функция ожидания готовности модуля по пину AUX
    // ================== ФУНКЦИИ RECEIVER ==================
      //===========================================================================================
        void setup_mcpwm  () {
          // Инициализация последовательного порта
          //    Serial. begin(115200);
          //  while(!Serial); // Для плат с USB-Serial
          
          Serial.println("\n\n=== Инициализация системы управления моторами ===");
          Serial.println("Модель: QX Motor QF(2827) 2227-1800KV");
          Serial.println("Тест: 4 минуты на 65% мощности\n");

          // Инициализация MCPWM
          init_motors();
          
          // Запуск тестовой последовательности
          run_test_sequence();
          
          Serial.println("\n=== Программа завершена _STOP_ ===");
        }

        void loop_mcpwm() {
          // После завершения теста ничего не делаем
          delay(1000);
        }

        // =============================================
        // ===     Функции инициализации    ===
        // =============================================

        /**
        * @brief Инициализация подсистемы MCPWM для управления моторами
        * 
        * Эта функция настраивает:
        * - Таймеры с заданной частотой и разрешением
        * - Операторы и компараторы
        * - Генераторы ШИМ сигналов
        * 
        * @note Все ошибки проверяются через ESP_ERROR_CHECK
        */
        void init_motors() {
          Serial.println("Инициализация MCPWM...");

          // 1. Конфигурация таймеров
          mcpwm_timer_config_t timer_config = {
            .group_id = 0,                      // Группа таймеров
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // Источник тактирования
            .resolution_hz = PWM_RESOLUTION_HZ,  // Разрешение таймера

            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,  // Режим счета (вверх)
            .period_ticks = PWM_PERIOD_TICKS,    // Период ШИМ

            .intr_priority = 0,       // явная инициализация
            .flags = {
                .update_period_on_empty  = false,      // Добавлено
                .update_period_on_sync = false,      // Добавлено
                .allow_pd = false                    // Добавлено
                }
              };  // end  timer_config

          // 2. Создание таймеров
          ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_a));
          ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_b));
          Serial.println("- Таймеры созданы");

          // 3. Конфигурация операторов
          mcpwm_operator_config_t operator_config = {
            .group_id = 0, // Та же группа, что и у таймеров
            .intr_priority = 0,                      // Добавлено
            .flags = {
                .update_gen_action_on_tez  = false,   // Добавлено
                .update_gen_action_on_tep  = false,   // Добавлено
                .update_gen_action_on_sync = false,   // Добавлено
                .update_dead_time_on_tez   = false,   // Добавлено
                .update_dead_time_on_tep   = false,   // Добавлено
                .update_dead_time_on_sync  = false    // Добавлено

            }
          }; //end  operator_config

          mcpwm_oper_handle_t oper_a = NULL;
          mcpwm_oper_handle_t oper_b = NULL;
          ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_a));
          ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_b));
          Serial.println("- Операторы созданы");

          // 4. Связывание операторов с таймерами
          ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_a, timer_a));
          ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_b, timer_b));
          Serial.println("- Операторы связаны с таймерами");

          // 5. Конфигурация компараторов
          mcpwm_comparator_config_t comparator_config = {
            .intr_priority = 0 ,  //  MCPWM comparator interrupt priority,
            .flags = {
                .update_cmp_on_tez = true,           // Добавлено
                .update_cmp_on_tep = false,          // Добавлено
                .update_cmp_on_sync = false,         // Добавлено
            }
          };  // end comparator_config
          //  .flags.update_cmp_on_tez = true, // Обновлять значение при обнулении таймера
          //  .flags.update_cmp_on_tep = true, // Whether to update compare value when timer count equals to peak (tep) 
          //  .flags.update_cmp_on_sync = true, // Whether to update compare value on sync event

          ESP_ERROR_CHECK(mcpwm_new_comparator(oper_a, &comparator_config, &comparator_a));
          ESP_ERROR_CHECK(mcpwm_new_comparator(oper_b, &comparator_config, &comparator_b));
          Serial.println("- Компараторы созданы");

          // 6. Установка начального значения (0% duty cycle)
          ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, 0));
          ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, 0));

          // 7. Конфигурация генераторов
          mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = MOTOR_A_PWM_PIN, // GPIO для мотора A
            .flags = {
                .invert_pwm   = false,                 // Добавлено
                .io_loop_back = false,               // Добавлено
                .io_od_mode   = false,               // Добавлено
                .pull_up      = false,               // Добавлено
                .pull_down    = false,               // Добавлено
            }
          };  // end generator_config

          ESP_ERROR_CHECK(mcpwm_new_generator(oper_a, &generator_config, &generator_a));
          generator_config.gen_gpio_num = MOTOR_B_PWM_PIN; // GPIO для мотора B
          ESP_ERROR_CHECK(mcpwm_new_generator(oper_b, &generator_config, &generator_b));
          Serial.println("- Генераторы созданы");

          // 8. Настройка действий генератора:
          // - Устанавливать HIGH при обнулении таймера
          // - Устанавливать LOW при срабатывании компаратора
          ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generator_a,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
          ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generator_a,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a, MCPWM_GEN_ACTION_LOW)));

          ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            generator_b,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
          ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            generator_b,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b, MCPWM_GEN_ACTION_LOW)));
          Serial.println("- Действия генераторов настроены");

          // 9. Запуск таймеров
          ESP_ERROR_CHECK(mcpwm_timer_enable(timer_a));
          //-----   ESP_ERROR_CHECK(mcpwm_timer_start(timer_a));    //error: 'mcpwm_timer_start' was not declared in this scope
          ESP_ERROR_CHECK(mcpwm_timer_enable(timer_b));
          //----    ESP_ERROR_CHECK(mcpwm_timer_start(timer_b));    // error: 'mcpwm_timer_start' was not declared in this scope
          Serial.println("- Таймеры запущены");
          Serial.println("Инициализация MCPWM завершена успешно!\n");
        }

        // =============================================
        // === Тестовая последовательность ===
        // =============================================

        /**
        * @brief Запуск тестовой последовательности
        * 
        * Последовательность работы:
        * 1. Плавный старт моторов до тестового уровня
        * 2. Работа на тестовом уровне в течение 4 минут
        * 3. Плавная остановка моторов
        * 
        * @note Весь процесс сопровождается выводом в Serial
        */
        void run_test_sequence() {
          Serial.println("=== Начало тестовой последовательности ===");
          
          // 1. Плавный старт
          Serial.println("\n [1/3] Плавный старт моторов...");
          smooth_start_motors();
          
          // 2. Основной тест
          Serial.println("\n [2/3] Основной тест: 65% мощности в течение 01 минут");
          set_motor_power(0, TEST_POWER_LEVEL);
          set_motor_power(1, TEST_POWER_LEVEL);
          
          unsigned long test_start = millis();
          while (millis() - test_start < TEST_DURATION_MS) {
            // Вывод оставшегося времени каждые 30 секунд
            static unsigned long last_print = 0;
            if (millis() - last_print >= 30000) {
              last_print = millis();
              unsigned long remaining = (TEST_DURATION_MS - (millis() - test_start)) / 1000;
              Serial.printf("Осталось: %lu мин %lu сек\n", remaining / 60, remaining % 60);
            }
            delay(100);
          }
          
          // 3. Плавная остановка
          Serial.println("\n [3/3] Плавная остановка моторов...");
          smooth_stop_motors();
          
          Serial.println("\n=== Тестовая последовательность завершена успешно! ===");
        }

        // =============================================
        // === Функции управления моторами ===
        // =============================================

        /**
        * @brief Плавный старт обоих моторов
        * 
        * Моторы плавно увеличивают мощность от 0% до тестового уровня
        * за время, указанное в SOFT_START_DURATION_MS
        */
        void smooth_start_motors() {
          Serial.print("Плавный разгон до ");
          Serial.print(TEST_POWER_LEVEL);
          Serial.println("%...");
          
          unsigned long start_time = millis();
          while (millis() - start_time < SOFT_START_DURATION_MS) {
            // Линейное увеличение мощности
            uint8_t power = map(millis() - start_time, 
                              0, SOFT_START_DURATION_MS, 
                              0, TEST_POWER_LEVEL);
            
            set_motor_power(0, power);
            set_motor_power(1, power);
            delay(10); // Небольшая задержка для плавности
          }
          
          Serial.println("Моторы вышли на рабочий режим");
        }

        /**
        * @brief Плавная остановка обоих моторов
        * 
        * Моторы плавно уменьшают мощность от текущего значения до 0%
        * за время, указанное в SOFT_START_DURATION_MS
        */
        void smooth_stop_motors() {
          Serial.println("Плавное снижение мощности до 0%...");
          
          uint8_t current_power = TEST_POWER_LEVEL;
          unsigned long start_time = millis();
          
          while (millis() - start_time < SOFT_START_DURATION_MS) {
            // Линейное уменьшение мощности
            uint8_t power = map(millis() - start_time, 
                              0, SOFT_START_DURATION_MS, 
                              current_power, 0);
            
            set_motor_power(0, power);
            set_motor_power(1, power);
            delay(10); // Небольшая задержка для плавности
          }
          
          // Гарантированная остановка
          set_motor_power(0, 0);
          set_motor_power(1, 0);
          
          Serial.println("Моторы полностью остановлены");
        }

        /**
        * @brief Установка мощности мотора
        * 
        * @param motor_num Номер мотора (0 - A, 1 - B)
        * @param power Уровень мощности (0-100%)
        * 
        * @note Функция автоматически ограничивает мощность в пределах MIN_DUTY_CYCLE-MAX_DUTY_CYCLE
        */ 
        void set_motor_power(uint8_t motor_num, uint8_t power) 
        {
          // Ограничение мощности
          power = constrain(power, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
          
          // Расчет рабочего цикла в тиках таймера
          uint32_t duty = (uint32_t)power * PWM_PERIOD_TICKS / 100;

          // Установка значения компаратора
          if (motor_num == 0) {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, duty));
          } else {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, duty));
          }
        }


/* 
//===================================================================================
//  moduleManager
//===================================================================================
   
void moduleMahager.initializeMotors(){
    init_motors();
}
void initializeEleron(){
  // Надо поставить угды в середину!

}
initializeSensors(){
  init ICM20948() ;
}
*/


/**
 * @brief Инициализация модуля EBYTE
 */
      //  void setup_ebyte_2_to_1() {
      void setup_ebyte() {
          ebyteStatus _es;    
          
          // Инициализация передатчика
          _es = sender.init();
          if (_es != EBYTE_SUCCESS) {
              Serial.print("Failed to initialize sender. Error: ");
              Serial.println(_es);
              while(1) delay(100);
          } else {
              Serial.println("Sender initialized successfully");
          }
          
          // Инициализация приемника
          _es = receiver.init();
          if (_es != EBYTE_SUCCESS) {
              Serial.print("Failed to initialize receiver. Error: ");
              Serial.println(_es);
              while(1) delay(100);
          } else {
              Serial.println("Receiver initialized successfully");
          }
          
          sender.enableDebug(true);
          receiver.enableDebug(true);
          
          Serial.println("EBYTE initialization complete");
      }

      /**
      * @brief Основной цикл обработки EBYTE
      */
          //  void loop_ebyte_2_to_1() {
      void loop_ebyte() {
          static uint32_t lastSendTime = 0;
          static uint32_t lastStatsTime = 0;
          static uint8_t consecutiveErrors = 0;
          
          // Отправка данных каждые 500 мс
          if (millis() - lastSendTime > 500) {
              testData.packet_id++;
              testData.timestamp = millis();
              
              ebyteStatus status = sender.sendData(testData);
              if (status != EBYTE_SUCCESS) {
                  consecutiveErrors++;
                  Serial.print("Send error: ");
                  Serial.println(status);
                  
                  if (consecutiveErrors > 3) {
                      Serial.println("Multiple consecutive errors, forcing recovery...");
                      sender.forceRecovery();
                      consecutiveErrors = 0;
                  }
              } else {
                  consecutiveErrors = 0;
              }
              
              lastSendTime = millis();
          }
          
          // Обработка входящих данных
          receiver.process();
          sender.process();
          
          // Вывод статистики каждые 5 секунд
          if (millis() - lastStatsTime > 5000) {
              Serial.println("Sender stats:");
              sender.printStats();
              
              Serial.println("Receiver stats:");
              receiver.printStats();
              
              lastStatsTime = millis();
          }
          
          delay(10);
      }

#ifdef LEDC_ELERON_MODULE
/**
 * @brief Настройка таймера LEDC для сервоприводов
 */
      void setupLEDCTimer() {
          ledc_timer_config_t timer_conf = {
              .speed_mode = LEDC_MODE,
              .duty_resolution = LEDC_DUTY_RES,
              .timer_num = LEDC_TIMER,
              .freq_hz = LEDC_FREQUENCY,
              .clk_cfg = LEDC_AUTO_CLK,
              .deconfigure = false
          };
          
          ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
      }

/**
 * @brief Настройка канала LEDC
 * @param chan_i Индекс канала
 * @param pin Пин GPIO
 */
      void setupLEDCChannel(int chan_i, int pin) {
          ledc_channel_config_t channel_conf = {
              .gpio_num = pin,
              .speed_mode = LEDC_MODE,
              .channel = channels[chan_i],
              .intr_type = LEDC_INTR_DISABLE,
              .timer_sel = LEDC_TIMER,
              .duty = 0,
              .hpoint = 0,
              .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
              .flags = {
                  .output_invert = false
              }
          };
          
          ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
      }

/**
 * @brief Инициализация системы управления элеронами
 */
      void Eleron_LEDCV2_setup() {
          setupLEDCTimer();
          
          // Настройка каналов для всех сервоприводов
          for (int i = 0; i < 5; i++) {
              setupLEDCChannel(i, servoPins[i]);
          }
          
          Serial.println("Eleron LEDC system initialized");
      }

/**
 * @brief Обработка команд управления с пульта
 */
      void proceed_air_command() {
          // Ограничение и установка целевых углов
          uint8_t tt = constrain(Com.comUp, 0, 180);
          targetAngles[1] = tt; 
          targetAngles[2] = tt; 
          
          tt = constrain(Com.comLeft, 0, 180);
          targetAngles[0] = tt; 
          targetAngles[3] = tt;
      }

/**
 * @brief Плавное обновление углов сервоприводов
 */
      void smoothUpdate() {
         // if(_debug.EBYTE_READY) {
              for (int i = 0; i < 5; i++) {
                  if (angles[i] < targetAngles[i]) {
                      angles[i]++;
                  } else if (angles[i] > targetAngles[i]) {
                      angles[i]--;
                  }
                  setServoAngle(i, angles[i]);
              }
         // }
      }

/**
 * @brief Установка угла сервопривода
 * @param chan_i Индекс канала
 * @param angle Угол (0-180 градусов)
 */
      void setServoAngle(int chan_i, int angle) {
          // Преобразование угла в ширину импульса
          int pulseWidth = map(angle, 0, 180, 500, 2400);
          
          // Преобразование в значение duty cycle
          uint32_t duty = (pulseWidth * (1 << LEDC_DUTY_RES)) / 20000;
          
          ledc_set_duty(LEDC_MODE, channels[chan_i], duty);
          ledc_update_duty(LEDC_MODE, channels[chan_i]);
          delay(ELERON_UPDATE_DELAY_MS);
      }

/**
 * @brief Основной цикл управления элеронами
 */
      void Eleron_LEDCV2_loop() {
          if ((millis() - eleron_test_count) < eleron_max_pause) {
              return;
          }
          
          if (eleron_count == 0) {
              proceed_air_command();
          } else if (eleron_count == 1) {
              smoothUpdate();
          }
          
          eleron_count = (eleron_count + 1) % 2;
          eleron_test_count = millis();
      }
#endif



/**
 * @brief Подготовка входящих данных
 */
      void prepare_income_data() {
          // Реализация подготовки данных
      }

/**
 * @brief Подготовка командных данных
 */
      void prepare_Com_data() {
          Com.comUp = 0;
          Com.comLeft = 0;
          Com.comThrottle = 3;
          Com.comSetAll = 0;
      }
//----------------------------------------------------------
// Функции для работы с ICM-20948

bool initICM20948() {
  /*
    Serial.println("Initializing ICM-20948...");
    //  1. Проверка питания и подключения 
    checkICMHardware() ;
    //  3.   Проверка сопротивления:
    Serial.println("\t ======== 3.   Проверка сопротивления:  ======");
    void checkResistance();
    //  2. Улучшенная проверка связи
    checkICMConnection_1();
    //  5. Проверка подключения:
    void checkICMConnection_2() ;    
    Serial.println("\t ================checkICMHardware()====================");
    //--------------------------------------------------
    */
    // 1. Инициализируем SPI
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, ICM_CS);
            //    pin Mode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH); // Деактивируем чип
    
    // 2. Проверяем наличие чипа
    digitalWrite(ICM_CS, LOW);
    SPI.transfer(0x00 | 0x80); // Чтение регистра WHO_AM_I
    uint8_t id = SPI.transfer(0x00);
    digitalWrite(ICM_CS, HIGH);
    
    Serial.printf("Chip ID: 0x%02X\n", id);
    if (id != 0xEA) {
        Serial.println("Wrong chip ID! Check wiring.");
        return false;
    }
    
    // 3. Инициализируем библиотеку
    if (myICM.begin(ICM_CS, SPI) != ICM_20948_Stat_Ok) {
        Serial.println("ICM-20948 initialization failed");
        return false;
    }
    
    Serial.println("ICM-20948 initialized successfully");
    return true;
}

  //  3.   Проверка сопротивления:
void checkResistance() {
    Serial.println("=== RESISTANCE CHECK ===");
    Serial.println("Measure with multimeter:");
    Serial.println("1. Resistance between MISO (GPIO19) and GND");
    Serial.println("   - Should be HIGH resistance (>10kΩ)");
    Serial.println("2. Resistance between MISO and 3.3V");
    Serial.println("   - Should be HIGH resistance");
    Serial.println("3. If resistance is low - SHORT CIRCUIT!");
}

/*
  //  1. Проверка питания и подключения
void checkICMHardware() {
    Serial.println("=== HARDWARE CHECK ===");
    
    // Проверка питания на пине CS
              //    pin Mode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);
    delay(100);
    
    // Измерение напряжения на пине CS (должно быть ~3.3V)
    Serial.printf("CS pin voltage: %s\n", digitalRead(ICM_CS) ? "HIGH (3.3V)" : "LOW (0V)");
    
    // Проверка всех пинов SPI
    int spiPins[] = {VSPI_MOSI, VSPI_MISO, VSPI_SCLK};
    const char* pinNames[] = {"MOSI (23)", "MISO (19)", "SCLK (18)"};
    
    for (int i = 0; i < 3; i++) {
                  //    pin Mode(spiPins[i], INPUT);
        int state = digitalRead(spiPins[i]);
        Serial.printf("%s: %d\n", pinNames[i], state);
    }
}
*/

/*
  //  5. Проверка подключения:
void checkICMConnection_2() {
    Serial.println("=== ICM-20948 Connection Test ===");
    
    // Проверка питания
            //pin Mode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);
    delay(100);
    
    // Проверка связи
    digitalWrite(ICM_CS, LOW);
    SPI.transfer(0x00 | 0x80); // WHO_AM_I запрос
    uint8_t response = SPI.transfer(0x00);
    digitalWrite(ICM_CS, HIGH);
    
    Serial.printf("WHO_AM_I response: 0x%02X\n", response);
    if (response == 0xEA) {
        Serial.println("✓ ICM-20948 connected properly");
    } else {
        Serial.println("✗ ICM-20948 connection failed");
        Serial.println("Check:");
        Serial.println("- VCC -> 3.3V");
        Serial.println("- GND -> GND");
        Serial.println("- SDA -> GPIO 23 (MOSI)");
        Serial.println("- SDO -> GPIO 19 (MISO)");
        Serial.println("- SCL -> GPIO 18 (SCK)");
        Serial.println("- CS -> GPIO 22");
    }
}

  //  2. Улучшенная проверка связи
bool checkICMConnection_1() {
    Serial.println("=== DETAILED CONNECTION TEST ===");
    
    // 1. Проверка питания
            //  pin Mode(ICM_CS, OUTPUT);
    digitalWrite(ICM_CS, HIGH);
    delay(100);
    
    // 2. Инициализация SPI
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, ICM_CS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    
    // 3. Несколько попыток чтения ID
    for (int attempt = 1; attempt <= 5; attempt++) {
        digitalWrite(ICM_CS, LOW);
        delayMicroseconds(10);
        
        SPI.transfer(0x00 | 0x80); // WHO_AM_I запрос
        uint8_t id = SPI.transfer(0x00);
        
        delayMicroseconds(10);
        digitalWrite(ICM_CS, HIGH);
        
        Serial.printf("Attempt %d: Chip ID = 0x%02X\n", attempt, id);
        
        if (id == 0xEA) {
            Serial.println("✓ ICM-20948 found!");
            return true;
        }
        
        delay(100);
    }
    
    Serial.println("✗ No response from ICM-20948");
    return false;
}
*/

  //  6.   Если используете DMP:
// В файле ICM_20948_C.h раскомментируйте:
//#define ICM_20948_USE_DMP
// Затем в коде:
bool initICM20948WithDMP_1() {
    if (myICM.begin(ICM_CS, SPI) != ICM_20948_Stat_Ok) {
        return false;
    }
    
    // Настройка DMP
    if (myICM.initializeDMP() != ICM_20948_Stat_Ok) {
        Serial.println("DMP initialization failed");
        return false;
    }
    
    // Включение нужных сенсоров
    myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
    myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER);
    
    return true;
}



      /*
      // Функции для работы с ICM-20948
      bool initSDCard_OLD() {
        if (!SD.begin(SD_CS)) {
          Serial.println("SD Card initialization failed!");
          return false;
        }
        Serial.println("SD Card initialized successfully");
        return true;
      }
      */

      /*
      bool initICM20948_OLD() {
        SPI.begin();  // Инициализируем SPI
        
        myICM.enableDebugging(Serial); // Отключаем отладку для производительности
        
        bool initialized = false;
        uint8_t attempts = 0;
        
        while (!initialized && attempts < 5) {
          Serial.print("Attempting to initialize ICM-20948, attempt ");
          Serial.println(attempts + 1);
          
          myICM.begin(ICM_CS, SPI);
          
          if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.println("ICM-20948 initialization failed");
            attempts++;
            delay(1000);
            continue;
          }
    
          // Настройка DMP
          bool success = true;
          success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
          success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
          success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
          success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
          success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
          
          if (success) {
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok);
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok);
            success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok);
          }
          
          if (success) {
            success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
            success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
            success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
            success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
          }
          
          if (success) {
            Serial.println("ICM-20948 DMP initialized successfully");
            initialized = true;
          } else {
            Serial.println("DMP initialization failed");
            attempts++;
            delay(1000);
          }
        }
        
        return initialized;
      } //  end  initICM20948_OLD() 
      */

      // =======================================================
      // Считаем углы Эйлера для всех направлений
      void calculateEulerAngles(float *quat, float *euler) {
        // Конвертация кватерниона в углы Эйлера (roll, pitch, yaw)
        float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
        
        // Roll (x-axis rotation)
        float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
        euler[0] = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
        
        // Pitch (y-axis rotation)
        float sinp = 2 * (q0 * q2 - q3 * q1);
        if (fabs(sinp) >= 1)
          euler[1] = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
        else
          euler[1] = asin(sinp) * 180.0 / M_PI;
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2 * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        euler[2] = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
      } // end  calculateEulerAngles(float *quat, float *euler) 

      void writeBinaryData(const SensorData &data) {
        dataFile.write((const uint8_t*)&data, sizeof(SensorData));
      }

      void readICMData() {
        icm_20948_DMP_data_t data;
        myICM.readDMPdataFromFIFO(&data);
        
        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
          SensorData sensorData;
          sensorData.timestamp = millis();
          
          if (data.header & DMP_header_bitmap_Quat6) {
            // Кватернион
            float q1 = ((float)data.Quat6.Data.Q1) / 1073741824.0;
            float q2 = ((float)data.Quat6.Data.Q2) / 1073741824.0;
            float q3 = ((float)data.Quat6.Data.Q3) / 1073741824.0;
            float q0 = sqrt(1.0 - (q1 * q1 + q2 * q2 + q3 * q3));
            
            sensorData.quat[0] = q0;
            sensorData.quat[1] = q1;
            sensorData.quat[2] = q2;
            sensorData.quat[3] = q3;
            
            // Расчет углов Эйлера
            calculateEulerAngles(sensorData.quat, sensorData.euler);
          }
          
          if (data.header & DMP_header_bitmap_Accel) {
            // Акселерометр
            sensorData.accel[0] = data.Raw_Accel.Data.X;
            sensorData.accel[1] = data.Raw_Accel.Data.Y;
            sensorData.accel[2] = data.Raw_Accel.Data.Z;
          }
          
          if (data.header & DMP_header_bitmap_Gyro) {
            // Гироскоп
            sensorData.gyro[0] = data.Raw_Gyro.Data.X;
            sensorData.gyro[1] = data.Raw_Gyro.Data.Y;
            sensorData.gyro[2] = data.Raw_Gyro.Data.Z;
          }
          
          if (data.header & DMP_header_bitmap_Compass) {
            // Магнитометр
            sensorData.mag[0] = data.Compass.Data.X;
            sensorData.mag[1] = data.Compass.Data.Y;
            sensorData.mag[2] = data.Compass.Data.Z;
          }
          
          sensorData.status = data.header;
          
          // Запись в файл
          if (dataFile) {
            writeBinaryData(sensorData);
          }
        }
      }// end readICMData()



            //  Модификация функции setup():
            //  cpp
bool initSDCard() {
    Serial.println("=== SD CARD DEBUG ===");
    Serial.print("CS pin: "); Serial.println(SD_CS);
    Serial.print("MOSI: "); Serial.println(VSPI_MOSI);
    Serial.print("MISO: "); Serial.println(VSPI_MISO);
    Serial.print("SCK: "); Serial.println(VSPI_SCLK);
    
    // Проверка пина CS
            //      pin Mode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    Serial.println("CS pin set to HIGH");
    
    // Тестирование SPI
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, SD_CS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    
    // Попробуйте разные скорости
    for (int speed = 1000000; speed >= 100000; speed -= 100000) {
        Serial.print("Trying SPI speed: "); Serial.println(speed);
        if (SD.begin(SD_CS, SPI, speed)) {
            Serial.println("Success!");
            return true;
        }
        delay(100);
    }
    
    return false;
}

    //  1. Добавьте функцию инициализации пинов в setup():
void initializeAllPins() {
    // Пины EBYTE
    pinMode(E49_PIN_M0, OUTPUT);
    pinMode(E49_PIN_M1, OUTPUT);
    pinMode(E49_PIN_AUX, INPUT);
    
    // SPI пины
    pinMode(VSPI_SCLK, OUTPUT);
    pinMode(VSPI_MISO, INPUT);
    pinMode(VSPI_MOSI, OUTPUT);
    pinMode(ICM_CS, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    
    // Пины сервоприводов
    for (int i = 0; i < 5; i++) {
        pinMode(servoPins[i], OUTPUT);
    }
    // Для моторных пинов GPIO32 и GPIO33 используйте:
    // Простое решение:
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

    // Светодиод
    pinMode(LED_PIN, OUTPUT);
    
    // Дополнительные пины
    pinMode(ICM_INT, INPUT);
    
    Serial.println("All pins initialized");
}// end  initializeAllPins()

    //  5. Добавьте проверку настройки пинов в отладочном режиме:
void debugPinStates() {
    if (_debug.setup_Debug) {
        Serial.println("=== PIN STATES ===");        
        Serial.println("5. Добавил проверку настройки пинов в отладочном режиме:");
        int pins[] = {E49_PIN_M0, E49_PIN_M1, E49_PIN_AUX, VSPI_SCLK, VSPI_MISO, VSPI_MOSI, ICM_CS, SD_CS, LED_PIN};
        const char* names[] = {"M0", "M1", "AUX", "SCLK", "MISO", "MOSI", "ICM_CS", "SD_CS", "LED"};
        
        for (int i = 0; i < 9; i++) {
                  //  pin Mode(pins[i], INPUT); // Временно как вход для чтения
            Serial.printf("%s: %d\n", names[i], digitalRead(pins[i]));
            // Возвращаем предыдущий режим
                    // if (i < 3) pin Mode(pins[i], OUTPUT);
                    //else if (i >= 3 && i <= 6) pin Mode(pins[i], OUTPUT);
                    //else if (i == 7) pin Mode(pins[i], OUTPUT);
                    //else pin Mode(pins[i], OUTPUT);
        }
    }
}


/**
 * @brief Настройка системы
 */
void setup() {

    //  if (_debug.setup_Debug)  Serial.println("START:Serial. begin(115200)");
    Serial.begin(115200);
    delay(1000);
    while (!Serial) delay(5);   // Для плат с USB-Serial  ЖДЕМ Готовности Порта
    if (_debug.setup_Debug)  {
        Serial.println("Serial. begin(115200)--Yet");
    }
    delay(550);
    
    if (_debug.setup_Debug)  Serial.println("START:Serial2.begin( 9600 , SERIAL_8N1, E49_PIN_RX, E49_PIN_TX);");
    Serial2.begin( 9600 , SERIAL_8N1, E49_PIN_RX, E49_PIN_TX);
    while (!Serial2)  delay(5);   // Для плат с USB-Serial  ЖДЕМ Готовности Порта
    if (_debug.setup_Debug)  {
        Serial.println("Serial2.begin");
    }
    delay (2000) ;


    if (_debug.setup_Debug){ 
        Serial.println("=== System Initialization ===");  
        Serial.println("=initializeAllPins()==");  
    }
    initializeAllPins();
    debugPinStates() ;
    delay(1000);  // Дайте питанию стабилизироваться


    /*   //------------------------------------------------------------------
    if (_debug.setup_Debug)  Serial.println("\t START: initSDCard()");
    // Инициализация SD карты
    if (!initSDCard()) {
      Serial.println("Failed to initialize SD card!");
      while(1);
    }
    if (_debug.setup_Debug)  {
        Serial.println("initSDCard()");
    }


    delay(1000);  // Дайте питанию стабилизироваться
    if (_debug.setup_Debug)  Serial.println("\t START: dataFile = SD.open");
    // Создание файла данных
    dataFile = SD.open("/sensor_data.bin", FILE_WRITE);
    if (!dataFile) {
      Serial.println("Failed to create data file!");
      while(1);
    }
    if (_debug.setup_Debug)  {
        Serial.println("dataFile = SD.open");
    }
    */

    if (_debug.setup_Debug)  Serial.println("\t START: initICM20948()");
    // Инициализация ICM-20948
    if (!initICM20948()) {
      Serial.println("Failed to initialize ICM-20948!");
      while(1);
    }
    if (_debug.setup_Debug)  {
        Serial.println("initICM20948()");
    }

    //--------------------------------------------------------------
    #ifdef EBYTE_MODULE
    if (_debug.setup_Debug)  Serial.println("\t START: setup_ebyte()");
    // Инициализация Модуля EBYTE
    setup_ebyte();
    if (_debug.setup_Debug)  {
        Serial.println("setup_ebyte()");
    }
    #endif

    #ifdef MCPWM_MOTOR_MODULE
    //    setup_mcpwm();
    if (_debug.setup_Debug)  {
        Serial.println("setup_mcpwm()");
    }
    #endif
    
    #ifdef LEDC_ELERON_MODULE
      Eleron_LEDCV2_setup();
    if (_debug.setup_Debug)  {
        Serial.println("Eleron_LEDCV2_setup()");
    }
    #endif
    
    loop_curent_state = 0;
      Serial.println("======  Setup complete   =======");
}

/**
 * @brief Основной цикл программы
 */
void loop() {
    // Выдержать паузу между циклами LOOP
    if ((millis() - main_loop_counter) < main_loop_counter_m_LIMIT) {
        return;
    }
    // EBYTE
    // Выдержать паузу между вызовами модуля EBYTE
    if( (millis() -  moduleTimers.lastEbyteTime ) > INTERVAL_EBYTE_PROCESS  ){
      // Знчит уже время работы EBYTE Модуля
      loop_ebyte();
      moduleTimers.lastEbyteTime  = millis() ;
    }

    // MCPWM
    // Выдержать паузу между вызовами модуля EBYTE
    if( (millis() -  moduleTimers.lastMotorTime ) > INTERVAL_MOTOR_UPDATE ){
      // Знчит уже время работы EBYTE Модуля
      loop_mcpwm();
      moduleTimers.lastMotorTime = millis();
    }

    // ELERON
    // Выдержать паузу между вызовами модуля EBYTE
    if( (millis() -  moduleTimers.lastEleronTime ) > INTERVAL_ELERON_UPDATE ){
      // Знчит уже время работы EBYTE Модуля
      Eleron_LEDCV2_loop();
    }

    // ICM_20948
    // Выдержать паузу между вызовами модуля EBYTE
    if( (millis() -  moduleTimers.lastSensorTime ) > INTERVAL_SENSOR_READ ){
      // Знчит уже время работы EBYTE Модуля
      readICMData();
      moduleTimers.lastSensorTime = millis();
    }
    main_loop_counter = millis(); // Начнем считать время цикля LOOP


    /*
    switch(loop_curent_state) {
        case 0:
            if(_debug.loop_Debug) {
                Serial.println("Main loop - CASE 0");
            }
            loop_curent_state++;
            break;
            
        case 1:
            if(_debug.loop_Debug) {
                Serial.println("Main loop - CASE 1");
            }
            loop_ebyte();
            loop_curent_state++;
            break;
            
        case 2:
            if(_debug.loop_Debug) {
                Serial.println("Main loop - CASE 2");
            }
            #ifdef LEDC_ELERON_MODULE
                Eleron_LEDCV2_loop();
            #endif
            loop_curent_state++;
            break;
            
        case 3:
          //  if(_debug.EBYTE_Debug && _debug.ELERON_Debug) {
            if(_debug.loop_Debug) {
                Serial.println("Main loop - CASE 3");
            }
            loop_curent_state++;
            break;
            
        case 4:
            if(_debug.loop_Debug) {
                Serial.println("Main loop - CASE 4");
            }

            loop_curent_state = 0;
            break;
    }
    
    main_loop_counter = millis();
    
    // Сброс счетчика при длительной работе
    if ((millis() - main_loop_counter) >= 5000) {
        main_loop_counter = millis();
    }
    */
}


