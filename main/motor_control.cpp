#include "motor_control.h"


void FOC_INIT()
{
    
    //初始化
    as5600.init();                                                   /*!< Enable as5600 */
    motor.linkSensor(&as5600);
    driver.voltage_power_supply = Voltage_supply;
    driver.voltage_limit = Voltage_limit;
    driver.init(0);
    motor.linkDriver(&driver);
    //velocity pid
    motor.PID_velocity.P = 0.9f;
    motor.PID_velocity.I = 0.05f;
    motor.PID_velocity.D = 0.5f;
    motor.voltage_limit = 11;
    motor.voltage_sensor_align = 3;
    motor.LPF_velocity.Tf = 0.3;
    motor.velocity_limit = 200;
    //position pid 
    motor.P_angle.P = 1.0;
    motor.P_angle.I = 0.05;
    motor.P_angle.D = 0.1;
    motor.LPF_angle.Tf = 0.1;
    //voltage pid
    motor.PID_current_q.P = 0.9;
    motor.PID_current_q.I = 0.1;
    motor.PID_current_q.D = 0.01;

    motor.init();                                                    /*!< Initialize motor */
    motor.initFOC();      

}

void Velocity_mode()
{
    motor.controller = MotionControlType::velocity;
    //  设置电机控制模式为速度控制模式
}
void Position_mode()
{
    motor.controller = MotionControlType::angle;
    //  设置电机控制模式为位置控制模式
}
void Torque_mode()
{
    motor.controller = MotionControlType::torque;
    // 设置电机控制模式为力矩控制模式
}





void FOC_start_loop(void* pvParameters)
{
    while(1)
    {
    motor.loopFOC();
    motor.move(target);
    ESP_LOGI("FOC","%f,%f",motor.shaft_angle,motor.shaft_velocity);
    vTaskDelay(pdMS_TO_TICKS(10));
    }

}


void fake_state_gen_EXHALE(void* pvParameters)
{   
    
    while(1)
    {
    int breathe_state=Exhale_state;
    ESP_LOGI("BREATHE","EXHALE");
    xQueueSend(breath_state_queue,&breathe_state, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void fake_state_gen_INHALE(void* pvParameters)
{   
    
    while(1)
    {
    int breathe_state=Inhale_state;
    ESP_LOGI("BREATHE","inHALE");
    xQueueSend(breath_state_queue,&breathe_state, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));

    }
}
void fake_state_gen(void* pvParameters)
{
    while(1)
    {
        xTaskCreatePinnedToCore(fake_state_gen_INHALE,"inhale ",4096,NULL,3,&inhale_handle,1);
        vTaskDelay(pdMS_TO_TICKS(3000));
        if(inhale_handle != NULL) 
        {
        vTaskDelete(inhale_handle);  // 删除指定任务
        inhale_handle = NULL;  // 任务句柄无效化
        }
        xTaskCreatePinnedToCore(fake_state_gen_EXHALE,"inhale ",4096,NULL,3,&exhale_handle,1);
        vTaskDelay(pdMS_TO_TICKS(8000));
        if(exhale_handle != NULL) 
        {
        vTaskDelete(exhale_handle);  // 删除指定任务
        exhale_handle = NULL;  // 任务句柄无效化
        }
    }
}


void app_breathe_train(void* pvParameters)
{
    
    int breathe_state=0;
    int pump_request=1;
    Torque_mode();
    while (1)
    {   

        motor.loopFOC();
        if(xQueueReceive(breath_state_queue,&breathe_state, pdMS_TO_TICKS(1000)))//接收队列中的数据
        {
            if(breathe_state==Exhale_state)//如果目前是呼气状态
            {
                
                set_pump_on();
                target=3;
                motor.move(target);
                vTaskDelay(pdMS_TO_TICKS(10));
                
            }
            if(breathe_state==Inhale_state)//如果目前是吸气状态
            {
                set_pump_off();
                target=0;
                motor.move(target);
                vTaskDelay(pdMS_TO_TICKS(10));
                
            }

        }
        else
        {
            target=0;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
}