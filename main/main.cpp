#include <main.h>









extern "C" void app_main(void)
{

    //init
    
    //FOC_INIT();//FOC初始化
    I2C_init();//I2C初始化
    MPU6050_init();//imu初始化
    servo_init();//舵机初始化
    Pump_Init();//气阀气泵初始化
      ADC_init();//ADC初始化
    // wifi_direct_connect();
    //queue create

    imu_data_queue = xQueueCreate(imu_queue_size, sizeof(imu_data));//imu数据传输队列初始化
    breath_state_queue=xQueueCreate(Breathe_queue_size ,sizeof(int));//呼吸状态队列初始化

    //sem create


    //app_task

    //xTaskCreatePinnedToCore(FOC_start_loop,"FOC ",4096,NULL,3,NULL,1);
    xTaskCreatePinnedToCore(app_imu_get_data,"imu ",4096,NULL,3,NULL,0);
    xTaskCreatePinnedToCore(app_data_receive,"imu ",4096,NULL,3,NULL,0);
    //xTaskCreatePinnedToCore(app_test_Servo,"servo ",4096,NULL,3,NULL,0);
    //xTaskCreatePinnedToCore(pump_test,"servo ",4096,NULL,3,NULL,0);
    //xTaskCreatePinnedToCore(fake_state_gen,"fakegen ",4096,NULL,3,NULL,1);
    //xTaskCreatePinnedToCore(app_breathe_train,"motortrain ",4096,NULL,3,NULL,1);



}
