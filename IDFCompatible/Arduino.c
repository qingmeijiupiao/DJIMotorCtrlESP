/*
 * @LastEditors: qingmeijiupiao
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-08-13 06:40:05
 */
#include "Arduino.h"


int64_t micros() {
  return esp_timer_get_time();
};

int64_t millis() {
  return esp_timer_get_time() / 1000ULL;
};

void delay(int ms){
    vTaskDelay(ms / portTICK_PERIOD_MS);
};