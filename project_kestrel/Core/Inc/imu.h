#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"

typedef struct {
    float roll;
    float pitch;
    float yaw_rate;
} Flight_Attitude_t;

void IMU_System_Init(void);
HAL_StatusTypeDef IMU_System_Update(void);
Flight_Attitude_t IMU_GetAttitude(void);

#endif