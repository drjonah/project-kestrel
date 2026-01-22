#ifndef BARO_H
#define BARO_H

#include "stm32f4xx_hal.h"

typedef struct {
    float altitude;
    float temperature;
} BARO_FlightData_t;

void BARO_System_Init(void);
HAL_StatusTypeDef BARO_System_Update(void);
BARO_FlightData_t BARO_GetAltitude(void);

#endif