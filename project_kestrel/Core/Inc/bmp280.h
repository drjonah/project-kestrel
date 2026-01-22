#ifndef BMP280_H
#define BMP280_H

#define BMP280_ADDR      0x76
#define REG_CTRL_MEAS    0xF4
#define REG_IIR_FILT     0xF5
#define REG_PRESS_MSB    0xF7
#define REG_CALIB_START  0x88

#define FS_P_ULP    1
#define FS_P_LP     2
#define FS_P_SP     3
#define FS_P_HP     4
#define FS_P_UHP    5

#define TEMP_OS_X1  1
#define TEMP_OS_X2  2
#define TEMP_OS_X4  3
#define TEMP_OS_X8  4
#define TEMP_OS_X16 5

#define FS_NORM_MODE    3

#define FS_IIR_0    1
#define FS_IIR_2    2
#define FS_IIR_4    3
#define FS_IIR_8    4
#define FS_IIR_16   5

#define CALIB_DATA_SIZE  24

extern long X_ACC_OFFSET;
extern long Y_ACC_OFFSET;
extern long Z_ACC_OFFSET;

extern long X_GYRO_OFFSET;
extern long Y_GYRO_OFFSET;
extern long Z_GYRO_OFFSET;

typedef struct {
    float temperature;
    float altitude;
} BMP280_Data_t;

typedef struct {
    int32_t raw_temperature;
    int32_t raw_pressure;
} BMP280_Raw_Data_t;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_Calib_Data_t;

typedef struct {
    int32_t temperature;
    uint32_t pressure;
} BMP280_Compensation_Data_t;

void bmp280_init();
void bmp280_calibrate(BMP280_Calib_Data_t *calib_data);
HAL_StatusTypeDef bmp280_read_t_p(BMP280_Data_t *baro_data, BMP280_Calib_Data_t calib_data);

#endif