#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

//CONST
#define Pump_gpio_num  GPIO_NUM_42
#define AirLock1_gpio_num  GPIO_NUM_41 //放气阀
#define AirLock2_gpio_num  GPIO_NUM_35  //充气阀
//QUEUE







void Pump_Init();
void set_pump_on(void);
void set_pump_off(void);
void pump_working(int mircosecond);
void pump_test(void* pvParameters);

