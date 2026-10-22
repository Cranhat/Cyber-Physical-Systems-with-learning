#include "Lidar.hpp"

Lidar::Lidar(uint16_t TxPin, uint16_t RxPin, int baudRate)
    : uart{TxPin, RxPin, baudRate, data, 2048}
{
}

Lidar::~Lidar()
{
    isReceiving = false;
}

int Lidar::receiveData()
{
    return uart.receive();
}
