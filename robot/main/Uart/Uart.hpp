#pragma once

#include <cstdint>
#include <driver/uart.h>

/*
Class to communicate rx and tx via uart.
*/

class Uart
{
public:
    Uart(uint16_t TxPin, uint16_t RxPin, int baudRate, uint8_t *data, uint16_t dataSize);
    ~Uart();

    void transmit();
    int receive();

    const int uart_buffer_size = (1024 * 2);

    uint8_t *data;
    uint16_t dataSize;

    QueueHandle_t uart_queue;
    const uart_port_t uart_num = UART_NUM_2;
};
