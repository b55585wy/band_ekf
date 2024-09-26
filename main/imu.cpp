#include "imu.h"



void I2C_init(void)
{
    i2c_config_t conf = {
        I2C_MODE_MASTER,               // 主机方式启动IIC
        I2C_MASTER_SDA_IO, 
        I2C_MASTER_SCL_IO,      // 设置数据引脚编号
        GPIO_PULLUP_ENABLE,   // 使能 SDA 上拉电阻
        GPIO_PULLUP_ENABLE,   // 使能 SCL 上拉电阻
        I2C_MASTER_FREQ_HZ // 设置主频
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "IIC 初始化完毕!");
}

void MPU6050_init()
{
    uint8_t check;
    /* 创建设备检测命令链，查看 0x75 寄存器返回的数据
     *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
     * ┌─┬─────────────┬─┬─┬───────────────┬─┬─┬─────────────┬─┬─┬───────────────┬─┬─┐
     * │S│  DEV-ADDR   │W│A│   WHO_AM_I    │A│S│  DEV-ADDR   │R│A│    RES DATA   │N│P│
     * └─┴────────────┴─┴─┴───────────────┴─┴─┴─────────────┴─┴─┴───────────────┴─┴─┘
     *  1      7        1 1        8        1 1       7       1 1        8        1 1
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送地址，以及写指令，命令之后需要带ACK
    i2c_master_write_byte(cmd, MPU_CMD_WHO_AM_I, true);                   // 发送 WHO_MI_I 寄存器地址 0x75
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &check, I2C_MASTER_LAST_NACK);              // 读取数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);                                             // 删除指令
    if (check != 0x68)
    {
        ESP_LOGE(TAG, "MPU6050 不在线!( %02X )", check);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 检测到在线，开始初始化...");
    /* 创建电源管理控制命令链，写入数据
     *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8
     * ┌─┬─────────────┬─┬─┬───────────────┬─┬───────────────┬─┬─┐
     * │S│  DEV-ADDR   │W│A│  PWR_MGMT_1   │A│   SEND DATA   │A│P│
     * └─┴─────────────┴─┴─┴───────────────┴─┴───────────────┴─┴─┘
     *  1      7        1 1        8        1        8        1 1
     */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_PWR_MGMT_1, true);                 // 写入电源管理和复位控制
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（设置陀螺仪采样率分频器）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_SMPLRT_DIV, true);                 // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x07, true);                               // 写入寄存器数据 Sample rate = 1kHz/(7+1) = 125Hz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（数字低通滤波器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_CONFIG, true);                     // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存数据 Gyroscope：260Hz 0ms，Accelerometer：256Hz 0.98ms 8Khz
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（陀螺仪配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_GYRO_CONFIG, true);                // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Gyroscope: +/- 250 dps
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    // 初始化默认参数（加速度传感器配置）
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_CONFIG, true);               // 写入寄存器地址
    i2c_master_write_byte(cmd, 0x00, true);                               // 写入寄存器数据 Accelerometer: +/- 2g
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG, "MPU6050 初始化完毕!");
}

int16_t get_accel_x()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_XOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    return data.value;
}

int16_t get_accel_y()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_YOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
    return data.value;
}

int16_t get_accel_z()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ACCEL_ZOUT_H, true);               // 写入寄存器地址，这个寄存是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
   return data.value;
}
int16_t get_angle_x()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ANGLE_XOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
   return data.value;
}
int16_t get_angle_y()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ANGLE_YOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数据
    i2c_cmd_link_delete(cmd);
   return data.value;
}
int16_t get_angle_z()
{
    union
    {
        uint8_t bytes[4];
        uint16_t value;
    } data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true); // 以写入方式发送地址
    i2c_master_write_byte(cmd, MPU_CMD_ANGLE_ZOUT_H, true);               // 写入寄存器地址，这个寄存器是加速度传感器 X轴 的高位地址
    i2c_master_start(cmd);                                                // 加入开始信号
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);  // 发送地址，以及读指令，命令之后需要带ACK
    i2c_master_read_byte(cmd, &data.bytes[1], I2C_MASTER_ACK);            // 读取高位字节数据，放在后面
    i2c_master_read_byte(cmd, &data.bytes[0], I2C_MASTER_NACK);           // 读取低位字节数据，放在前面
    i2c_master_stop(cmd);                                                 // 加入停止信号
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));       // 开始发送数
    i2c_cmd_link_delete(cmd);
   return data.value;
}
imu_data imu_send_wifi()
{
    imu_data imu;
    imu.x_acc=get_accel_x();
    imu.y_acc=get_accel_y();
    imu.z_acc=get_accel_z();
    imu.x_anl=get_angle_x();
    imu.y_anl=get_angle_y();
    imu.z_anl=get_angle_z();
    return imu;
}

float twoscomplementscale(int16_t data, int8_t scale, int16_t range=32767)
{
    float data_f;
    // printf("data:%d\n", data);
    data_f = data * 1.0 / range * scale;
    // printf("1: %f\n",data * 1.0 / float(range));
    // printf("2: %f\n",data * 1.0 / float(range) * scale);
    // printf("converted to:%f\n", data_f);
    return data_f;
}

#define FILTER_ORDER 5

// 初始化FIR滤波器
void init_fir_filter(FIRFilter* filter) {
    // 使用给定的滤波器系数
    filter->coeffs[0] = 0.0356953f;
    filter->coeffs[1] = 0.24106222f;
    filter->coeffs[2] = 0.44648496f;
    filter->coeffs[3] = 0.24106222f;
    filter->coeffs[4] = 0.0356953f;

    // 初始化滤波器缓冲区为0
    memset(filter->buffer, 0, sizeof(float) * FILTER_ORDER);
    // 初始化缓冲区索引为0
    filter->index = 0;
}

// 应用FIR滤波器
float apply_fir_filter(FIRFilter* filter, float input) {
    // 将新的输入放入缓冲区
    filter->buffer[filter->index] = input;
    
    float output = 0;
    int index = filter->index;
    
    // 计算滤波后的输出
    for (int i = 0; i < FILTER_ORDER; i++) {
        output += filter->coeffs[i] * filter->buffer[index];
        index = (index - 1 + FILTER_ORDER) % FILTER_ORDER;
    }
    
    // 更新索引
    filter->index = (filter->index + 1) % FILTER_ORDER;
    
    return output;
}

#define STATE_DIM 3
#define MEASURE_DIM 1

typedef struct {
    float x[STATE_DIM];  // State: [position, velocity, acceleration]
    float P[STATE_DIM][STATE_DIM];  // Covariance matrix
    float Q[STATE_DIM][STATE_DIM];  // Process noise covariance
    float R;  // Measurement noise covariance
} EKF;

static EKF ekf;

// 初始化扩展卡尔曼滤波器
void init_ekf() {
    // 初始化状态
    memset(ekf.x, 0, sizeof(ekf.x));
    
    // 初始化协方差矩阵
    memset(ekf.P, 0, sizeof(ekf.P));
    // 调整初始不确定性，考虑到肚子起伏的幅度通常很小
    ekf.P[0][0] = 0.01;  // 位置初始不确定性（单位：米）
    ekf.P[1][1] = 0.001; // 速度初始不确定性（单位：米/秒）
    ekf.P[2][2] = 0.1;   // 加速度初始不确定性（单位：G）
    
    // 初始化过程噪声协方差
    memset(ekf.Q, 0, sizeof(ekf.Q));
    ekf.Q[0][0] = 1e-6;  // 位置过程噪声
    ekf.Q[1][1] = 1e-5;  // 速度过程噪声
    ekf.Q[2][2] = 1e-4;  // 加速度过程噪声
    
    // 初始化测量噪声协方差
    ekf.R = 0.01;  // 加速度测量噪声（单位：G^2）
}

// EKF预测步骤
void ekf_predict(float dt) {
    // 状态转移矩阵
    // 状态转移矩阵
    float F[STATE_DIM][STATE_DIM] = {
        {1, dt, 0.5f*dt*dt},
        {0, 1, dt},
        {0, 0, 1}
    };
    
    // 预测状态
    float x_pred[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        x_pred[i] = F[i][0]*ekf.x[0] + F[i][1]*ekf.x[1] + F[i][2]*ekf.x[2];
    }
    memcpy(ekf.x, x_pred, sizeof(ekf.x));
    
    // 预测协方差
    float FP[STATE_DIM][STATE_DIM], FPFT[STATE_DIM][STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            FP[i][j] = F[i][0]*ekf.P[0][j] + F[i][1]*ekf.P[1][j] + F[i][2]*ekf.P[2][j];
        }
    }
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            FPFT[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2];
        }
    }
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            ekf.P[i][j] = FPFT[i][j] + ekf.Q[i][j];
        }
    }
}

// EKF更新步骤
void ekf_update(float measurement) {
    // 测量矩阵
    float H[MEASURE_DIM][STATE_DIM] = {{0, 0, 1}};
    
    // 计算新息
    float y = measurement - ekf.x[2];
    
    // 计算新息协方差
    float S = H[0][0]*ekf.P[0][2] + H[0][1]*ekf.P[1][2] + H[0][2]*ekf.P[2][2] + ekf.R;
    
    // 计算卡尔曼增益
    float K[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        K[i] = (ekf.P[i][0]*H[0][0] + ekf.P[i][1]*H[0][1] + ekf.P[i][2]*H[0][2]) / S;
    }
    
    // 更新状态
    for (int i = 0; i < STATE_DIM; i++) {
        ekf.x[i] += K[i] * y;
    }
    
    // 更新协方差
    float IKH[STATE_DIM][STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            IKH[i][j] = (i == j) ? 1.0f - K[i]*H[0][j] : -K[i]*H[0][j];
        }
    }
    
    float IKHP[STATE_DIM][STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            IKHP[i][j] = IKH[i][0]*ekf.P[0][j] + IKH[i][1]*ekf.P[1][j] + IKH[i][2]*ekf.P[2][j];
        }
    }
    memcpy(ekf.P, IKHP, sizeof(ekf.P));
}

#define ZERO_VELOCITY_THRESHOLD 0.01  // 单位：m/s
#define DISPLACEMENT_THRESHOLD 0.05  // 单位：米 假设最高起伏是5cm

// 估计位移
float estimate_displacement(float acceleration) {
    static bool initialized = false;
    static float last_time = 0;
    static float last_zero_velocity_time = 0;
    float current_time = esp_timer_get_time() / 1000000.0f;  // 转换为秒
    float dt = current_time - last_time;
    last_time = current_time;

    if (!initialized) {
        init_ekf();
        initialized = true;
    }

    ekf_predict(dt);
    ekf_update(acceleration);

    bool is_zero_velocity = fabsf(ekf.x[1]) < ZERO_VELOCITY_THRESHOLD;

    if (is_zero_velocity) {
        // 检查是否在顶部或底部
        if (fabsf(ekf.x[0]) > DISPLACEMENT_THRESHOLD) {
            // 在顶部或底部，执行部分重置
            ekf.x[1] = 0;  // 将速度设置为零
            // 稍微增加位置的不确定性
            ekf.P[0][0] += 0.001;
        } else {
            // 在中间位置，执行完全重置
            ekf.x[1] = 0;  // 将速度设置为零
            ekf.x[0] = 0;  // 将位置重置为零
            // 重置位置和速度的不确定性
            ekf.P[0][0] = 0.01;
            ekf.P[1][1] = 0.001;
        }
        last_zero_velocity_time = current_time;
    } else if (current_time - last_zero_velocity_time > 0.5f) {
        // 如果超过0.5秒没有检测到零速度，稍微增加不确定性
        ekf.P[0][0] += 0.0001;
        ekf.P[1][1] += 0.0001;
    }

    return ekf.x[0];  // 返回估计的位移
}

void app_imu_get_data(void* pvParameters)
{
    imu_data imu;
    FIRFilter x_acc_filter, y_acc_filter, z_acc_filter;
    FIRFilter x_anl_filter, y_anl_filter, z_anl_filter;
    
    init_fir_filter(&x_acc_filter);
    init_fir_filter(&y_acc_filter);
    init_fir_filter(&z_acc_filter);
    init_fir_filter(&x_anl_filter);
    init_fir_filter(&y_anl_filter);
    init_fir_filter(&z_anl_filter);

    vTaskDelay(100);    // 等待一秒钟开始获取数据
    while (1)
    {
        int16_t x_acc = get_accel_x();
        int16_t y_acc = get_accel_y();
        int16_t z_acc = get_accel_z();
        int16_t x_anl = get_angle_x();
        int16_t y_anl = get_angle_y();
        int16_t z_anl = get_angle_z();

        if (USE_FILTER) {
            imu.x_acc = apply_fir_filter(&x_acc_filter, twoscomplementscale(x_acc, 2, 32767));
            imu.y_acc = apply_fir_filter(&y_acc_filter, twoscomplementscale(y_acc, 2, 32767));
            imu.z_acc = apply_fir_filter(&z_acc_filter, twoscomplementscale(z_acc, 2, 32767));
            imu.x_anl = apply_fir_filter(&x_anl_filter, twoscomplementscale(x_anl, 250, 32767));
            imu.y_anl = apply_fir_filter(&y_anl_filter, twoscomplementscale(y_anl, 250, 32767));
            imu.z_anl = apply_fir_filter(&z_anl_filter, twoscomplementscale(z_anl, 250, 32767));
        } else {
            imu.x_acc = twoscomplementscale(x_acc, 2, 32767);
            imu.y_acc = twoscomplementscale(y_acc, 2, 32767);
            imu.z_acc = twoscomplementscale(z_acc, 2, 32767);
            imu.x_anl = twoscomplementscale(x_anl, 250, 32767);
            imu.y_anl = twoscomplementscale(y_anl, 250, 32767);
            imu.z_anl = twoscomplementscale(z_anl, 250, 32767);
        }

        // Estimate displacement
        float displacement = estimate_displacement(imu.z_acc);

        // Add the displacement to the imu struct
        imu.displacement = displacement;

        xQueueSend(imu_data_queue, &imu, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_data_receive(void* pvParameters)
{
    imu_data imu;
        while (1)
        {
        if (xQueueReceive(imu_data_queue, &imu, portMAX_DELAY)) 
        {
            // printf("%d,%d,%d,%d,%d,%d\n", imu.x_acc,imu.y_acc,imu.z_acc,imu.x_anl,imu.y_anl,imu.z_anl);
            printf("displacement: %f\n", imu.displacement);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
         }
}   

