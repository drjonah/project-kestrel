# Project Kestrel
Drone flight software and ground communication software built from scratch.

# Project State

## Complete 
### MCU (STM32 Black Pill)
- Integrate MPU6050 with Kalman filter
- Integrate BMP280

### Raspberry Pi 5
- Dockerize environment with hardware access
- Setup PS4 controller handling (pyPS4Controller)
- Setup NRF24L01 transceiver listening

## In Progress
### MCU (STM32 Black Pill)
- Implement NRF24L01 SPI driver
- Receive control packets from Pi

### Raspberry Pi 5
- Transmit control packets to MCU

## Next Steps
### MCU (STM32 Black Pill)
- Integrate motor mixing logic (PWM)

## Raspberry Pi 5
- Implement telemetry data logging
