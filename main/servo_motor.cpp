#include "servo_motor.h"




void servo_init()//初始化舵机定时器
{
        // 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        LEDC_MODE,
        LEDC_DUTY_RES,
         LEDC_TIMER,
        LEDC_FREQUENCY,  // 设置频率为200Hz
        LEDC_AUTO_CLK,
        false // 初始化所有的成员
    };
    ledc_timer_config(&ledc_timer);

    // 配置LEDC通道
    ledc_channel_config_t ledc_channel = {
        LEDC_OUTPUT_IO,
        LEDC_MODE,
        LEDC_CHANNEL,
        LEDC_INTR_DISABLE,
        LEDC_TIMER,
        0, // 初始占空比为0
        0,
        0 // 初始化所有的成员
    };
    ledc_channel_config(&ledc_channel);

}


uint32_t servo_per_degree_init(float degree_of_rotation) //将角度转换为pwm占空比
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth=degree_of_rotation*40/180+10;//转换为百分比制的占空比
    cal_pulsewidth=cal_pulsewidth*8191/100;//将百分比制的占空比转换为13bit分辨率的ledc控制模块占空比，占空比= cal_pulsewidth*100%/(2^13-1)
    return cal_pulsewidth;
}






void set_angle_to_servo(float angle)//支持0~180deg
{
    uint32_t duty=servo_per_degree_init(angle);//根据转角得到占空比
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);//设置占空比
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);//更新占空比
}

void app_test_Servo(void* pvParameters)
{
    while (1) {
        // 循环设置舵机角度
        for (float angle = 0; angle <= SERVO_MAX_DEGREE; angle++) {
            set_angle_to_servo(angle);
            ESP_LOGI("SERVO","%f",angle);
            vTaskDelay(100 / portTICK_PERIOD_MS); // 延迟20ms
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒
        for (float angle = SERVO_MAX_DEGREE; angle >= 0; angle--) {
            set_angle_to_servo(angle);
            ESP_LOGI("SERVO","%f",angle);
            vTaskDelay(100 / portTICK_PERIOD_MS); // 延迟20ms
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒
    }
}