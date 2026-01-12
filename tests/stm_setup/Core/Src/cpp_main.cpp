#include "main.h"

#include <cstdio> // Include for printf

// This function acts as the bridge. 
// 'extern "C"' tells the C++ compiler to generate a C-compatible function name.
extern "C" void cpp_main() {
    
    // You can now write standard C++ here!
    // Example: DroneController myDrone;
    // myDrone.init();

    while (1) {
        printf("Status: Armed | Voltage: 12.4V\r\n");

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        HAL_Delay(100);

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        HAL_Delay(100);

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        HAL_Delay(200);
    }
}