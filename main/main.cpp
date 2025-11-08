#include "UartDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main(void)
{
    UartDriver uart(&UART1, GPIO_NUM_25, GPIO_NUM_26, 115200);
    uart.Init();

    uart.WriteString("C++ UART Driver Ready!\r\n");

    uint8_t rxChar;
    while (true)
    {
        if (uart.ReadByte(rxChar))
        {
            uart.WriteString("Echo: ");
            uart.WriteByte(rxChar);
            uart.WriteString("\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
