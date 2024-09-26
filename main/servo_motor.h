#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>
#include "driver/ledc.h"
#include "esp_log.h"

#define Servo_motor_gpio GPIO_NUM_46

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (Servo_motor_gpio) // 定义舵机控制引脚
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // PWM占空比的分辨率
#define LEDC_FREQUENCY          (200) // 频率设置为50Hz，适合舵机控制
#define SERVO_MIN_PULSEWIDTH 500  // 微秒，舵机最小脉宽对应最小角度（通常为 0 度）
#define SERVO_MAX_PULSEWIDTH 2500 // 微秒，舵机最大脉宽对应最大角度（通常为 180 度）
#define SERVO_MAX_DEGREE 180      // 舵机最大角度



void servo_init();//初始化舵机
uint32_t servo_per_degree_init(uint32_t degree_of_rotation);
void set_angle_to_servo(float angle);
void app_test_Servo(void* pvParameters);