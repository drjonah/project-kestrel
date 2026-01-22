#include "baro.h"
#include "bmp280.h" 
#include "kalman.h" 
#include "stm32f4xx_hal.h"
#include <math.h>

// Private variables
static BMP280_Calib_Data_t bmp_calib_data;
static BMP280_Data_t bmp_data;
static BARO_FlightData_t current_altitude;

void BARO_System_Init(void) {
    bmp280_init(); // Initialize hardware
    bmp280_calibrate(&bmp_calib_data); // Calibrate the BMP280
}

HAL_StatusTypeDef BARO_System_Update(void) {
    HAL_StatusTypeDef ret;

    // Read Hardware
    ret = bmp280_read_t_p(&bmp_data, bmp_calib_data);
    if (ret != HAL_OK) {
        return ret;
    }

    current_altitude.altitude = bmp_data.altitude;
    current_altitude.temperature = bmp_data.temperature;

    // Return success
    return HAL_OK;
}

BARO_FlightData_t BARO_GetAltitude(void) {
    return current_altitude;
}