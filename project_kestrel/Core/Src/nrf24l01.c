#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "nrf24l01.h"
#include "main.h"

// --- NRF24L01+ Register Map ---
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

// --- Instruction Set ---
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8 
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define NOP           0xFF


void NRF24_Init() {
    // Disable chip to configure
    NRF24_CE_LOW;
    HAL_Delay(10);

    // Configure address width with 5 bytes
    NRF24_WriteReg(SETUP_AW, 0x03);

    // Set RF channel / frequency
    // 76 = 1476 MHz (must match the receiver)
    NRF24_WriteReg(RF_CH, 76);

    // Configure pipes
    uint8_t address[] = {0x30, 0x30, 0x30, 0x30, 0x31};
    NRF24_WriteRegMulti(RX_ADDR_P0, address, 5);
    NRF24_WriteRegMulti(TX_ADDR, address, 5);

    // Enable auto-ack
    NRF24_WriteReg(EN_AA, 0x01);

    // Enable dynamic payloads
    NRF24_WriteReg(DYNPD, 0x01);
    NRF24_WriteReg(FEATURE, 0x06); 

    // Clear old data
    NRF24_FlushRX();
    NRF24_FlushTX();

    // Power up and setup receiver (ref specs)
    NRF24_WriteReg(CONFIG, 0x0F);

    // Start listening
    NRF24_CE_HIGH;
}

uint8_t NRF24_Check() {
    // Prepare a test (5 bytes)
    uint8_t tx_buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t rx_buf[5] = {0x00};

    // Save the current address so we don't break the config
    uint8_t backup_addr[5];
    NRF24_ReadRegMulti(TX_ADDR, backup_addr, 5);

    // Write the test pattern
    NRF24_WriteRegMulti(TX_ADDR, tx_buf, 5);

    // Read it back
    NRF24_ReadRegMulti(TX_ADDR, rx_buf, 5);

    // Restore the original address
    NRF24_WriteRegMulti(TX_ADDR, backup_addr, 5);

    // Compare the read vs written
    for (int i = 0; i < 5; i++) {
        if (rx_buf[i] != tx_buf[i]) {
            printf("[NRF 24] Transceiver check failure! \n");
            return 0; // FAIL
        }
    }
    
    printf("[NRF 24] Transceiver check success! \n");
    return 1; 
}

// // Receive logic
// uint8_t NRF24_DataAvailable() {

// }

// void NRF24_ReadPayload() {

// }

// Transmit logic
void NRF24_WriteAckPayload(uint8_t pipe, Telemetry_Data_t *data) {

}

// Interrupt 
void NRF24_IRQ_Handler() {

}

// Helpers
void NRF24_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = reg | W_REGISTER;
    buf[1] = value;

    // Pull CSN low to start transaction
    NRF24_CSN_LOW;
    // Send the Register address + Value
    HAL_SPI_Transmit(NRF24_SPI, buf, 2, 100);
    // Pull CSN high to finish
    NRF24_CSN_HIGH;
}

void NRF24_WriteRegMulti(uint8_t reg, uint8_t *data, uint8_t size) {
    uint8_t cmd = reg | W_REGISTER;
    
    NRF24_CSN_LOW;
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100); // Send Command
    HAL_SPI_Transmit(NRF24_SPI, data, size, 100); // Send Data
    NRF24_CSN_HIGH;
}

void NRF24_ReadRegMulti(uint8_t reg, uint8_t *data, uint8_t size) {
    uint8_t cmd = reg | R_REGISTER; // "Read Command" (0x00)
    
    NRF24_CSN_LOW;
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100); // Send Command
    HAL_SPI_Receive(NRF24_SPI, data, size, 100); // Read Data
    NRF24_CSN_HIGH;
}

void NRF24_SendCommand(uint8_t cmd) {
    NRF24_CSN_LOW;
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);
    NRF24_CSN_HIGH;
}

void NRF24_FlushRX(void) {
    NRF24_SendCommand(FLUSH_RX);
}

void NRF24_FlushTX(void) {
    NRF24_SendCommand(FLUSH_TX);
}
