#include <stdio.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include <string.h>
#include"driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_simplefoc.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include <unistd.h>
#include "esp_mac.h"
#include "sdkconfig.h"

//子模块
#include "motor_control.cpp"
#include "imu.cpp"
#include "servo_motor.cpp"
#include "pump.cpp"
#include "wifi.c"
#include "adc.c"