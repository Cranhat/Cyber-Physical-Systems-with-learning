#include "Uart.hpp"

Uart::Uart(uint16_t TxPin, uint16_t RxPin, int baudRate, uint8_t* data, uint16_t dataSize)
    : data(data), dataSize(dataSize)
{

    uart_config_t uart_config = {}; 

    uart_config.baud_rate = baudRate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 100;
    uart_config.source_clk = UART_SCLK_APB;
    uart_config.flags.allow_pd = false;
    uart_config.flags.backup_before_sleep = false;
    uart_config.rx_glitch_filt_thresh = 0;

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TxPin, RxPin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

Uart::~Uart() = default;

void Uart::transmit()
{
}

int Uart::receive()
{
    size_t available = 0;
    uart_get_buffered_data_len(uart_num, &available);

    if (available > 0)
    {
        return uart_read_bytes(uart_num, data, available, 1000);
    }
    return 0;
}

// Packet Uart::receiveDataToSend(){
//     size_t available = 0;
//     uart_get_buffered_data_len(uart_num, &available);

//     if (available > 0) {
//         length = uart_read_bytes(uart_num, data, available, 1000);
//     } else {
//         length = 0;
//     }
//     return Packet(data, length);
// }
