



        void setup_new_icm_OLD() {
                    //Serial.begin(115200);
                    //while (!Serial) delay(50);
                    
                    //Serial2.begin(9600, SERIAL_8N1, E49_PIN_RX, E49_PIN_TX);
                    //while (!Serial2);
          delay(2000);
          
          Serial.println("=== System Initialization ===");
          
          // Инициализация SD карты
          if (!initSDCard()) {
            Serial.println("Failed to initialize SD card---!");
            while(1);
          }
          
          // Создание файла данных
          dataFile = SD.open("/sensor_data.bin", FILE_WRITE);
          if (!dataFile) {
            Serial.println("Failed to create data file!");
            while(1);
          }
          
          // Инициализация ICM-20948
          if (!initICM20948()) {
            Serial.println("Failed to initialize ICM-20948!");
            while(1);
          }
          
          setup_ebyte();
          
          #ifdef LEDC_ELERON_MODULE
          Eleron_LEDCV2_setup();
          #endif
          
          loop_curent_state = 0;
          Serial.println("Setup complete");
        }

            //  Модификация основного цикла:
            //  cpp

      void loop_new_icm__OLD_NOT_USE() {
        static uint32_t lastICMRead = 0;
        const uint32_t ICM_READ_INTERVAL = 10; // 100Hz
        
        if ((millis() - main_loop_counter) < main_loop_counter_m_LIMIT) {
          return;
        }
        
        // Чтение данных с ICM-20948
        if ( (millis() - lastICMRead) > ICM_READ_INTERVAL) {
          readICMData();
          lastICMRead = millis();
        }
        
        switch(loop_curent_state) {
          // ... остальные case без изменений
        }
        
        // Периодическое сохранение файла
        static uint32_t lastFlushTime = 0;
        if (millis() - lastFlushTime > 5000) {
          dataFile.flush();
          lastFlushTime = millis();
          Serial.println("Data flushed to SD card");
        }
        
        main_loop_counter = millis();
      }

      //Ответ на вопрос: Возможно ли это?
      //  ===================================================



