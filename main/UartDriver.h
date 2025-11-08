#pragma once
#include "driver/gpio.h"
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "hal/gpio_hal.h"
#include "soc/io_mux_reg.h"
#include <cstring>

class UartDriver
{
private:
    uart_dev_t* uartPort;
    gpio_num_t txPin;
    gpio_num_t rxPin;
    uint32_t baudRate;
    static constexpr uint32_t UARTClockFreq = APB_CLK_FREQ;  // renamed here ✅

public:
    UartDriver(uart_dev_t* port, gpio_num_t tx, gpio_num_t rx, uint32_t baud)
        : uartPort(port), txPin(tx), rxPin(rx), baudRate(baud) {}

    void Init()
    {
        // Enable peripheral clock
        if (uartPort == &UART0) periph_module_enable(PERIPH_UART0_MODULE);
        else if (uartPort == &UART1) periph_module_enable(PERIPH_UART1_MODULE);
        else if (uartPort == &UART2) periph_module_enable(PERIPH_UART2_MODULE);

        // Configure pins
        esp_rom_gpio_pad_select_gpio(txPin);
        esp_rom_gpio_pad_select_gpio(rxPin);
        gpio_set_direction(txPin, GPIO_MODE_OUTPUT);
        gpio_set_direction(rxPin, GPIO_MODE_INPUT);

        // Connect to UART hardware signals
        if (uartPort == &UART1) {
            esp_rom_gpio_connect_out_signal(txPin, U1TXD_OUT_IDX, false, false);
            esp_rom_gpio_connect_in_signal(rxPin, U1RXD_IN_IDX, false);
        } else if (uartPort == &UART2) {
            esp_rom_gpio_connect_out_signal(txPin, U2TXD_OUT_IDX, false, false);
            esp_rom_gpio_connect_in_signal(rxPin, U2RXD_IN_IDX, false);
        } else {
            esp_rom_gpio_connect_out_signal(txPin, U0TXD_OUT_IDX, false, false);
            esp_rom_gpio_connect_in_signal(rxPin, U0RXD_IN_IDX, false);
        }

        // Reset config
        uartPort->conf0.val = 0;
        uartPort->conf1.val = 0;

        // Set baud rate
        uint32_t clk_div = UARTClockFreq / baudRate;   // use renamed constant ✅
        uartPort->clk_div.div_int = clk_div;

        // 8 data bits, no parity, 1 stop bit
        uartPort->conf0.bit_num = 3;          // 8 bits
        uartPort->conf0.stop_bit_num = 1;     // 1 stop
        uartPort->conf0.parity_en = 0;        // no parity

        ESP_LOGI("UART", "Custom UART Initialized: TX=%d RX=%d Baud=%lu",
                 txPin, rxPin, baudRate);
    }

    void WriteByte(uint8_t data)
    {
        while (uartPort->status.txfifo_cnt >= 127);
        uartPort->fifo.rw_byte = data;
    }

    void WriteString(const char* str)
    {
        while (*str)
            WriteByte(static_cast<uint8_t>(*str++));
    }

    bool ReadByte(uint8_t& data)
    {
        if (uartPort->status.rxfifo_cnt > 0)
        {
            data = uartPort->fifo.rw_byte;
            return true;
        }
        return false;
    }
};
