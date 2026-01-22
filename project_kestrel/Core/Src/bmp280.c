#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "bmp280.h"
#include "main.h"
#include "stm32f4xx_hal_i2c.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c2;


void bmp280_init() {
    HAL_Delay(2000);

    // Check if BMP 280 is connected
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, BMP280_ADDR << (1 + 0), 1, 100);
    if (ret == HAL_OK) {
        printf("[BMP 280] The device is ready. \n");
    }
    else {
        printf("[BMP 280] The device is not ready. Check cables \n");
    }

    // Configure BMP 280 measurements
    uint8_t ctrl_meas_reg = 0;

    // Temperature config (bits 5-7)
    ctrl_meas_reg |= (TEMP_OS_X1 << 5);

    // Pressure config (bits 2-4)
    ctrl_meas_reg |= (FS_P_UHP << 2);

    // Power config (bits 0-1)
    ctrl_meas_reg |= FS_NORM_MODE;

    // Write measurement config 
    ret = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDR << (1 + 0), REG_CTRL_MEAS, 1, &ctrl_meas_reg, 1, 100);
    if (ret == HAL_OK) {
        printf("[BMP 280] Configured barometer measurements \n");
    }
    else {
        printf("[BMP 280] Failed to configure barometer measurements \n");
    }

    // Configure BMP 280 IIR filter
    uint8_t irr_filter_reg = 0;

    // Config IIR filter (bits 2-4)
    irr_filter_reg |= (FS_IIR_16 << 2);

    // Write IIR config
    ret = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDR << (1+ 0), REG_IIR_FILT, 1, &irr_filter_reg, 1, 100);
    if (ret == HAL_OK) {
        printf("[BMP 280] Configured barometer filters \n");
    }
    else {
        printf("[BMP 280] Failed to configure barometer filters \n");
    }
}


HAL_StatusTypeDef bmp280_read_raw(BMP280_Raw_Data_t *raw_data_out) {
    uint8_t raw_data[6];

    // Read data
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR << (1+ 0), REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, raw_data, 6, 100);

    if (ret == HAL_OK) {
        // --- Reassembling the Data (20-bit format) ---
        // The BMP280 outputs data as: [MSB] [LSB] [XLSB]

        // 1. Construct Raw Pressure (from raw_data[0], [1], [2])
        //    Format: MSB << 12 | LSB << 4 | XLSB >> 4
        int32_t adc_p = (int32_t)(raw_data[0] << 12) | 
                        (int32_t)(raw_data[1] << 4)  | 
                        (int32_t)(raw_data[2] >> 4);

        // 2. Construct Raw Temperature (from raw_data[3], [4], [5])
        //    Format: MSB << 12 | LSB << 4 | XLSB >> 4
        int32_t adc_t = (int32_t)(raw_data[3] << 12) | 
                        (int32_t)(raw_data[4] << 4)  | 
                        (int32_t)(raw_data[5] >> 4);
        
        raw_data_out->raw_temperature = adc_t;
        raw_data_out->raw_pressure    = adc_p;
    } else {
        printf("[BMP280] Read failed \n");
    }

    return ret;
}


void bmp280_calibrate(BMP280_Calib_Data_t *calib_data) {
    uint8_t raw_calib[CALIB_DATA_SIZE]; // Buffer for 24 bytes

    // Read 24 bytes starting from 0x88
    HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR << (1 + 0), REG_CALIB_START, I2C_MEMADD_SIZE_8BIT, raw_calib, CALIB_DATA_SIZE, 100);

    // Combine the bytes (Little Endian: LSB is first, MSB is second)
    
    // Temperature
    calib_data->dig_T1 = (uint16_t)((raw_calib[1] << 8) | raw_calib[0]);
    calib_data->dig_T2 = (int16_t) ((raw_calib[3] << 8) | raw_calib[2]);
    calib_data->dig_T3 = (int16_t) ((raw_calib[5] << 8) | raw_calib[4]);

    // Pressure
    calib_data->dig_P1 = (uint16_t)((raw_calib[7] << 8) | raw_calib[6]);
    calib_data->dig_P2 = (int16_t) ((raw_calib[9] << 8) | raw_calib[8]);
    calib_data->dig_P3 = (int16_t) ((raw_calib[11] << 8)| raw_calib[10]);
    calib_data->dig_P4 = (int16_t) ((raw_calib[13] << 8)| raw_calib[12]);
    calib_data->dig_P5 = (int16_t) ((raw_calib[15] << 8)| raw_calib[14]);
    calib_data->dig_P6 = (int16_t) ((raw_calib[17] << 8)| raw_calib[16]);
    calib_data->dig_P7 = (int16_t) ((raw_calib[19] << 8)| raw_calib[18]);
    calib_data->dig_P8 = (int16_t) ((raw_calib[21] << 8)| raw_calib[20]);
    calib_data->dig_P9 = (int16_t) ((raw_calib[23] << 8)| raw_calib[22]);
    
    printf("[BMP280] Calibration data loaded \n");
}


BMP280_Compensation_Data_t bmp280_read_compensation(BMP280_Calib_Data_t calib_data, BMP280_Raw_Data_t raw_data) {
    // Formulas: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf, Page 22

    BMP280_Compensation_Data_t compensation_data = {0, 0};

    // --- Temperature Compensation ---
    int32_t t_var1, t_var2, t_fine;

    t_var1 = (((raw_data.raw_temperature >> 3) - (calib_data.dig_T1 << 1)) * calib_data.dig_T2) >> 11;
    t_var2 = (((((raw_data.raw_temperature >> 4) - calib_data.dig_T1) * ((raw_data.raw_temperature >> 4) - calib_data.dig_T1)) >> 12) * calib_data.dig_T3) >> 14;
    t_fine = t_var1 + t_var2;
    compensation_data.temperature = (t_fine * 5 + 128) >> 8;

    // --- Pressure Compensation ---
    int64_t p_var1, p_var2, p;

    p_var1 = t_fine - 128000;
    p_var2 = p_var1 * p_var1 * calib_data.dig_P6;
    p_var2 = p_var2 + (p_var1 * calib_data.dig_P5 << 17);
    p_var2 = p_var2 + (((int64_t) calib_data.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t) calib_data.dig_P3) >> 8) + ((p_var1 * (int64_t) calib_data.dig_P2) << 12);
    p_var1 = ((((int64_t) 1) << 47) + p_var1) * calib_data.dig_P1 >> 33;

    if (p_var1 == 0) {
        compensation_data.pressure = 0;
    }
    else {
        p = 1048576 - raw_data.raw_pressure;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (calib_data.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (calib_data.dig_P8 * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (calib_data.dig_P7 << 4);

        compensation_data.pressure = p;
    }


    return compensation_data;
}


HAL_StatusTypeDef bmp280_read_t_p(BMP280_Data_t *baro_data, BMP280_Calib_Data_t calib_data) {
    BMP280_Raw_Data_t raw_data;

    HAL_StatusTypeDef ret = bmp280_read_raw(&raw_data);
    if (ret == HAL_OK) {
        BMP280_Compensation_Data_t data = bmp280_read_compensation(calib_data, raw_data);

        // Store temperature
        baro_data->temperature = (float) data.temperature / 100.0f;

        // Store pressure
        float pressure_pa = (float) data.pressure / 256.0f;
        baro_data->altitude = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
    }

    return ret;
}

