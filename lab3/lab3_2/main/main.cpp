#include "lcd.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


void main_func(void){
  LCDDisplayManager lcd;
    lcd.init();
    while (true) {
      lcd.setRGB(255, 255, 255);
      vTaskDelay(100);
      lcd.printstr((char *) "Hello CSE121!");
      vTaskDelay(100);
      lcd.setCursor(0, 1);
      vTaskDelay(100);
      lcd.printstr((char *)"Shreyas");
      vTaskDelay(100);
      lcd.setCursor(0,0);
      vTaskDelay(3000);
    }
}

extern "C" void app_main(void){
    main_func();
}