#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/queue.h"
#include "pump.h"
#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif

#define POLE_NUM 11
#define Resisitance 7
#define Motor_U_gpio GPIO_NUM_36
#define Motor_V_gpio GPIO_NUM_37
#define Motor_W_gpio GPIO_NUM_38
#define Motor_EN_gpio GPIO_NUM_40
#define Voltage_supply 12
#define Voltage_limit 6

#define Inhale_state 1//吸气
#define Exhale_state 0//呼气





float target = 0 ;
BLDCMotor motor = BLDCMotor(POLE_NUM,Resisitance);
BLDCDriver3PWM driver = BLDCDriver3PWM(36, 37, 38,40);
AS5600 as5600 = AS5600(I2C_NUM_1, GPIO_NUM_4, GPIO_NUM_5);
//queue
QueueHandle_t breath_state_queue;//呼吸状态
QueueHandle_t delete_queue;//清除任务指令
QueueHandle_t ADC_ALarm_queue;//ADC阈值过高触发报警
#define Breathe_queue_size 32
//handle
TaskHandle_t exhale_handle = NULL;
TaskHandle_t inhale_handle = NULL;





void FOC_INIT();//FOC初始化
void Velocity_mode();//设置速度模式
void Position_mode();//设置位置模式
void Torque_mode();//设置力矩模式
void FOC_start_loop();
void app_breathe_train();//训练应用
