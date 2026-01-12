#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

long X_ACC_OFFSET = 0;
long Y_ACC_OFFSET = 0;
long Z_ACC_OFFSET = 0;

long X_GYRO_OFFSET = 0;
long Y_GYRO_OFFSET = 0;
long Z_GYRO_OFFSET = 0;

void mpu6050_init()
{
    HAL_Delay(5000);

    // Check if MPU 6050 is connected
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS << (1 + 0), 1, 100);
    if (ret == HAL_OK) {
        printf("The device is ready.\n");
    }
    else {
        printf("The device is not ready. Check cables \n");
    }

    // Configure MPU 6050
    // gyro
    uint8_t temp_data = FS_GYRO_500;
    ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << (1 + 0), REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
    if (ret == HAL_OK) {
        printf("Configuring gyroscope.\n");
    }
    else {
        printf("Failed to configure gyroscope.\n");
    }

    // acc
    temp_data = FS_ACC_4G;
    ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << (1 + 0), REG_CONFIG_ACC, 1, &temp_data, 1, 100);
    if (ret == HAL_OK) {
        printf("Configuring accelerometer.\n");
    }
    else {
        printf("Failed to configure the accelerometer.\n");
    }

    // sleep
    temp_data = 0;
    ret = HAL_I2C_Mem_Write(&hi2c1, DEVICE_ADDRESS << (1 + 0), REG_USR_CTRL, 1, &temp_data, 1, 100);
    if (ret == HAL_OK) {
        printf("Exiting from sleep mode.\n");
    }
    else {
        printf("Failed to exit from sleep mode.\n");
    }
}

void mpu6050_calibrate()
{
    MPU6050_Data_t acc_data;
    MPU6050_Data_t gyro_data;
    
    long x_acc_sum = 0;
    long y_acc_sum = 0;
    long z_acc_sum = 0;
    long x_gyro_sum = 0;
    long y_gyro_sum = 0;
    long z_gyro_sum = 0;

    printf("Calibrating IMU... Keep stationary.\n");

    for (uint16_t i = 0; i < 2000; i++) 
    {
        // 2. We read the sensor. 
        // NOTE: Because our global offsets are currently 0 (at startup),
        // mpu6050_read_acc returns the raw data, which is exactly what we want here.
        if(mpu6050_read_acc(&acc_data) == HAL_OK && mpu6050_read_gyro(&gyro_data) == HAL_OK)
        {
            x_acc_sum += acc_data.x;
            y_acc_sum += acc_data.y;
            z_acc_sum += acc_data.z;

            x_gyro_sum += gyro_data.x;
            y_gyro_sum += gyro_data.y;
            z_gyro_sum += gyro_data.z;
        }
        else
        {
            printf("Sensor Read Error during calibration!\r\n");
        }

        HAL_Delay(2);
    }

    X_ACC_OFFSET = x_acc_sum / 2000;
    Y_ACC_OFFSET = y_acc_sum / 2000;
    Z_ACC_OFFSET = (z_acc_sum / 2000) - ACCEL_1G_LSB; 

    X_GYRO_OFFSET = x_gyro_sum / 2000;
    Y_GYRO_OFFSET = y_gyro_sum / 2000;
    Z_GYRO_OFFSET = z_gyro_sum / 2000;

    printf("Calibration Complete.\r\n");
    printf("Gyro Offsets: X:%ld Y:%ld Z:%ld\r\n", X_ACC_OFFSET, Y_ACC_OFFSET, Z_ACC_OFFSET);
    printf("Acc Offsets: X:%ld Y:%ld Z:%ld\r\n", X_GYRO_OFFSET, Y_GYRO_OFFSET, Z_GYRO_OFFSET);
}

HAL_StatusTypeDef mpu6050_read_acc(MPU6050_Data_t *acc_data)
{
    uint8_t data[6];

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_ACC, 1, data, 6, 100);

    if (ret == HAL_OK) {
        acc_data->x = (int16_t)(data[0] << 8 | data[1]) - X_ACC_OFFSET;
        acc_data->y = (int16_t)(data[2] << 8 | data[3]) - Y_ACC_OFFSET;
        acc_data->z = (int16_t)(data[4] << 8 | data[5]) - Z_ACC_OFFSET;
    }

    return ret;
}

HAL_StatusTypeDef mpu6050_read_gyro(MPU6050_Data_t *gyro_data)
{
    uint8_t data[6];

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, REG_DATA_GYRO, 1, data, 6, 100);

    if (ret == HAL_OK) {
        gyro_data->x = (int16_t)(data[0] << 8 | data[1]) - X_GYRO_OFFSET;
        gyro_data->y = (int16_t)(data[2] << 8 | data[3]) - Y_GYRO_OFFSET;
        gyro_data->z = (int16_t)(data[4] << 8 | data[5]) - Z_GYRO_OFFSET;
    }

    return ret;
}
