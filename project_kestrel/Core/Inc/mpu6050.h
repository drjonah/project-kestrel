#ifndef INC_MPU6050_H
#define INC_MPU6050_H


#define DEVICE_ADDRESS  0x68

#define FS_GYRO_250     0
#define FS_GYRO_500     8
#define FS_GYRO_1000    9
#define FS_GYRO_2000    10

#define FS_ACC_2G       0
#define FS_ACC_4G       8
#define FS_ACC_8G       9
#define FS_ACC_16G      10
#define ACCEL_1G_LSB    8192

#define REG_CONFIG_GYRO     27
#define REG_CONFIG_ACC      28
#define REG_USR_CTRL        107
#define REG_DATA_ACC        59
#define REG_DATA_GYRO       67

extern long X_ACC_OFFSET;
extern long Y_ACC_OFFSET;
extern long Z_ACC_OFFSET;

extern long X_GYRO_OFFSET;
extern long Y_GYRO_OFFSET;
extern long Z_GYRO_OFFSET;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_Data_t;

void mpu6050_init();
void mpu6050_calibrate();
HAL_StatusTypeDef mpu6050_read_acc(MPU6050_Data_t *acc_data);
HAL_StatusTypeDef mpu6050_read_gyro(MPU6050_Data_t *gyro_data);


#endif
