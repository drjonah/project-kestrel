#ifndef INC_NRF24L01_H
#define INC_NRF24L01_H

#include <stdint.h>
#include <stdbool.h>

// SPI Bus
extern SPI_HandleTypeDef hspi1; 
#define NRF24_SPI &hspi1

// CE, CSN, IQR Pins
#define NRF24_CE_PORT   GPIOA
#define NRF24_CE_PIN    GPIO_PIN_3
#define NRF24_CSN_PORT  GPIOA
#define NRF24_CSN_PIN   GPIO_PIN_4
#define NRF24_IRQ_PORT  GPIOA
#define NRF24_IRQ_PIN   GPIO_PIN_0

// Pin Macros
#define NRF24_CE_LOW    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET)
#define NRF24_CE_HIGH   HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET)
#define NRF24_CSN_LOW   HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET)
#define NRF24_CSN_HIGH  HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET)

typedef struct {
    float throttle;
    float pitch;
    float roll;
    bool arm_status;
} Control_Data_t; // From Pi

typedef struct {
    float voltage;
    float altitude;
    float orientation;
} Telemetry_Data_t; // To Pi

void NRF24_Init();
uint8_t NRF24_Check();

// Receive logic
uint8_t NRF24_DataAvailable();
void NRF24_ReadPayload();

// Transmit logic
void NRF24_WriteAckPayload(uint8_t pipe, Telemetry_Data_t *data);

// Interrupt 
void NRF24_IRQ_Handler();

// Helpers
void NRF24_WriteReg(uint8_t reg, uint8_t value);
void NRF24_WriteRegMulti(uint8_t reg, uint8_t *data, uint8_t size);
void NRF24_ReadRegMulti(uint8_t reg, uint8_t *data, uint8_t size);
void NRF24_SendCommand(uint8_t cmd);
void NRF24_FlushRX(void);
void NRF24_FlushTX(void);

#endif