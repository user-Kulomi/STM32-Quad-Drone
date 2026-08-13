#include "int_mpu6050.h"

/**
 * @brief 向MPU6050指定寄存器写入数据
 * 
 * @param reg 寄存器地址
 * @param data 写入数据
 */
void Int_MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    // 使用HAL库的I2C写入函数向MPU6050指定寄存器写入数据
    //参数1：I2C句柄，参数2：从设备地址，参数3：寄存器地址(带有读写位)，参数4：寄存器地址位数（8位）
    //参数5：要写入的数据的首地址（支持写入多字节），参数6：数据长度，参数7：超时写入时间
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

/**
 * @brief 从MPU6050指定寄存器读取数据
 * 
 * @param reg 寄存器地址
 * @param data 存放读取到的数据的首地址
 */
void Int_MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    // 使用HAL库的I2C读取函数从MPU6050指定寄存器读取数据
    //参数1：I2C句柄，参数2：从设备地址，参数3：寄存器地址(带有读写位)，参数4：寄存器地址位数（8位）
    //参数5：存放读取到的数据的首地址（支持读取多字节），参数6：数据长度，参数7：超时读取时间
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

int16_t Acc_x_offset = 0;
int16_t Acc_y_offset = 0;
int16_t Acc_z_offset = 0;

int16_t Gyro_x_offset = 0;
int16_t Gyro_y_offset = 0;
int16_t Gyro_z_offset = 0;

/**
 * @brief 在完成MPU6050初始化的末尾进行零偏校准
 */
void Int_MPU6050_calculate_offset(void)
{
    //1.等待飞机放置平稳

    //判断依据：加速度三个轴的数据跳动范围持续小于STABLE_COND_FLUCTUATION_ALLOW_VAL_ACC，达到100次，即代表放置平稳
    Acc_struct last_data = {0};
    Acc_struct current_data = {0};
    uint8_t count = 0;

    //获取加速度到last_data:
    Int_MPU6050_Get_Acc(&last_data);

    //轮询数值波动小于允许值STABLE_COND_FLUCTUATION_ALLOW_VAL_ACC的次数，直到大于100次:
    while(count < 100)
    {
        Int_MPU6050_Get_Acc(&current_data);
        if(abs(current_data.accel_x - last_data.accel_x) < STABLE_COND_FLUCTUATION_ALLOW_VAL_ACC &&
           abs(current_data.accel_y - last_data.accel_y) < STABLE_COND_FLUCTUATION_ALLOW_VAL_ACC && 
           abs(current_data.accel_z - last_data.accel_z) < STABLE_COND_FLUCTUATION_ALLOW_VAL_ACC )
        {
            count++;
        }
        else
        {
            count = 0;
        }
        last_data = current_data;
        vTaskDelay(6);//延时6ms，避免任务阻塞
    }

    //2.计算零偏值，便于后续调用：

    //六个轴的累计偏差值：
    int32_t Acc_x_offset_sum = 0;
    int32_t Acc_y_offset_sum = 0;
    int32_t Acc_z_offset_sum = 0;

    int32_t Gyro_x_offset_sum = 0;
    int32_t Gyro_y_offset_sum = 0;
    int32_t Gyro_z_offset_sum = 0;

    //存储六轴数据:
    Acc_struct acc_data = {0};
    Gyro_struct gyro_data = {0};

    for (uint8_t i = 0; i < 100; i++)//测量100次取平均值
    {
        //实时获取六轴数据：
        Int_MPU6050_Get_Acc(&acc_data);
        Int_MPU6050_Get_Gyro(&gyro_data);

        Acc_x_offset_sum += (acc_data.accel_x - 0);
        Acc_y_offset_sum += (acc_data.accel_y - 0);
        //Z轴加速度初值应为0.98g。按±2g量程换算，对应的16位数据为 32768 * 0.98 / 2 = 16056
        Acc_z_offset_sum += (acc_data.accel_z - 16056);
        
        Gyro_x_offset_sum += (gyro_data.gyro_x - 0);
        Gyro_y_offset_sum += (gyro_data.gyro_y - 0);
        Gyro_z_offset_sum += (gyro_data.gyro_z - 0);

        vTaskDelay(6);//延时6ms
    }
    //最终的偏差值：
    Acc_x_offset = Acc_x_offset_sum / 100;
    Acc_y_offset = Acc_y_offset_sum / 100;
    Acc_z_offset = Acc_z_offset_sum / 100;

    Gyro_x_offset = Gyro_x_offset_sum / 100;
    Gyro_y_offset = Gyro_y_offset_sum / 100;
    Gyro_z_offset = Gyro_z_offset_sum / 100;
}


/**
 * @brief 初始化MPU6050
 * 
 */
void Int_MPU6050_Init(void)
{
    //1.重启芯片：

    //1.1 重启MPU6050芯片:

    //通过写电源管理寄存器1的设备复位(0x80)位来重置所有寄存器值，从而重启MPU6050芯片:
    Int_MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x80);

    //1.2 等待重启完成：

    //通过判断电源管理寄存器1的值是否为0x40来确认芯片是否已经重启完成（重启完成，0x6B值会变为0x40，芯片睡眠）:
    uint8_t reg_value = 0;
    while (reg_value != 0x40)
    {
        Int_MPU6050_Read_Reg(MPU_PWR_MGMT1_REG, &reg_value);
    }

    //1.3 唤醒MPU6050使之进入到正常工作状态：

    //将电源管理寄存器1的睡眠位(0x40)清零，使MPU6050从睡眠模式唤醒，进入正常工作状态:
    Int_MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x00);

    //2.设置合适量程（尽量选择够用范围内尽可能小的量程，保证精准）：

    //2.1 设置陀螺仪量程为 ±2000°/s：
    Int_MPU6050_Write_Reg(MPU_GYRO_CFG_REG, 0x18);

    //2.2 设置加速度计量程为 ±2g：
    Int_MPU6050_Write_Reg(MPU_ACCEL_CFG_REG, 0x00);

    //3.关闭中断使能，因为用不到：
    Int_MPU6050_Write_Reg(MPU_INT_EN_REG, 0x00);

    //4.设置用户配置寄存器为0，因为用不到里面的FIFO和拓展I2C功能：
    Int_MPU6050_Write_Reg(MPU_USER_CTRL_REG, 0x00);

    //5.设置采样率：
    //根据香农定理，采样率必须大于等于使用频率的2倍，否则会失真，由于该函数对应的任务周期为6ms，即1s内
    //会使用1000 / 6 ≈ 166.67次数据，因此采样率必须大于等于2 * 166.67 ≈ 333.34Hz，选择采样率为500Hz即可满足要求
    //采样率 = 陀螺仪输出率 / (1 + SMPLRT_DIV)，而陀螺仪默认输出率=1000Hz，因此设置SMPLRT_DIV = 1即可得到500Hz的采样率:
    Int_MPU6050_Write_Reg(MPU_SAMPLE_RATE_REG, 0x01);

    //6.设置低通滤波器：
    //将加速度计和角速度计的低通滤波值分别设为184Hz与188Hz，往0x1A寄存器(低通滤波器)写入0x01即可：
    Int_MPU6050_Write_Reg(MPU_CFG_REG, 1);

    //7.选择使用的系统时钟：
    //选择一个添加了PLL（倍频器）的时钟作为时钟源：
    Int_MPU6050_Write_Reg(MPU_PWR_MGMT1_REG, 0x01);// 写0x01表示选择X轴陀螺仪的PLL作为时钟源

    //8.使能加速度计和陀螺仪：
    Int_MPU6050_Write_Reg(MPU_PWR_MGMT2_REG, 0x00);

    //9.计算并更新零偏值：
    Int_MPU6050_calculate_offset();
}

/**
 * @brief 读取三轴角速度（存在轻微抖动，需要进行零偏校准）
 * 
 * @param gyro_data 存放角速度数据的结构体指针
 */
void Int_MPU6050_Get_Gyro(Gyro_struct *gyro_data)
{
    uint8_t high_byte = 0;
    uint8_t low_byte = 0;// 用于存放读取到的高8位和低8位数据

    //读取X轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &high_byte);
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low_byte);
    gyro_data -> gyro_x = (high_byte << 8 | low_byte) - Gyro_x_offset;// 将原始数据拼接，并减去零偏值
    
    // 读取Y轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &high_byte);
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low_byte);
    gyro_data -> gyro_y = (high_byte << 8 | low_byte) - Gyro_y_offset;
    
    // 读取Z轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &high_byte);
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low_byte);
    gyro_data -> gyro_z = (high_byte << 8 | low_byte) - Gyro_z_offset;

}

/**
 * @brief 读取三轴加速度（抖动很严重，需要进行校准。Z轴值不为0）
 * 
 * @param acc_data 存放加速度数据的结构体指针
 */
void Int_MPU6050_Get_Acc(Acc_struct *acc_data)
{
    uint8_t high_byte = 0;
    uint8_t low_byte = 0;// 用于存放读取到的高8位和低8位数据
    
    //读取X轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &high_byte);
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low_byte);
    acc_data -> accel_x = (high_byte << 8 | low_byte) - Acc_x_offset; // 将原始数据拼接，并减去零偏值
    
    // 读取Y轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &high_byte);
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low_byte);
    acc_data -> accel_y = (high_byte << 8 | low_byte) - Acc_y_offset;
    
    // 读取Z轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &high_byte);
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low_byte);
    acc_data -> accel_z = (high_byte << 8 | low_byte) - Acc_z_offset;
}

/**
 * @brief 读取所有的六轴数据
 * 
 * @param gyro_acc_data 共同存放角速度和加速度数据的结构体指针
 */
void Int_MPU6050_Get_Data(Gyro_Accel_struct *gyro_acc_data)
{
    Int_MPU6050_Get_Gyro(&gyro_acc_data -> gyro_data);
    Int_MPU6050_Get_Acc(&gyro_acc_data -> acc_data);
}



