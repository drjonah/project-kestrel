#include "imu.h"
#include "mpu6050.h" 
#include "kalman.h" 
#include "stm32f4xx_hal.h"
#include <math.h>

// Private variables
static Kalman_t kalmanRoll;
static Kalman_t kalmanPitch;
static MPU6050_Data_t accData;
static MPU6050_Data_t gyroData;
static Flight_Attitude_t currentAttitude;
static uint32_t last_time;

void IMU_System_Init(void) {
    mpu6050_init(); // Initialize hardware
    mpu6050_calibrate(); // Calibrate the MPU6050
    Kalman_Init(&kalmanRoll); // Initialize Kalman roll filter
    Kalman_Init(&kalmanPitch); // Initialize Kalman pitch filter
    last_time = HAL_GetTick();
}

HAL_StatusTypeDef IMU_System_Update(void) {
    HAL_StatusTypeDef ret;

    // Read Hardware
    ret = mpu6050_read_acc(&accData);
    if (ret != HAL_OK) {
        return ret;
    }
    ret = mpu6050_read_gyro(&gyroData);
    if (ret != HAL_OK) {
        return ret;
    }

    // Calculate dt
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    last_time = now;

    // Calculate roll & pitch angles
    float roll_acc = atan2f(accData.y, accData.z) * 180.0f / M_PI;
    float pitch_acc = atan2f(-accData.x, sqrtf(accData.y*accData.y + accData.z*accData.z)) * 180.0f / M_PI;

    // Calculate roll & pitch rates
    float roll_rate = gyroData.x / 65.5f;
    float pitch_rate = gyroData.y / 65.5f;

    // Calculate roll, pitch, & yaw
    currentAttitude.roll = Kalman_GetAngle(&kalmanRoll, roll_acc, roll_rate, dt);
    currentAttitude.pitch = Kalman_GetAngle(&kalmanPitch, pitch_acc, pitch_rate, dt);
    currentAttitude.yaw_rate = gyroData.z / 65.5f;

    // Return success
    return HAL_OK;
}

Flight_Attitude_t IMU_GetAttitude(void) {
    return currentAttitude;
}