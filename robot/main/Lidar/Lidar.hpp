#pragma once

#include "LidarFrame.hpp"
#include "Uart/Uart.hpp"
#include <cstdint>
#include <chrono>

/*
Class to get data from LiDAR sensor, and convert it
into packages that can be sent further.
*/

// The frames are already constructed so do we really
// need this structs or just blindly transmit them to PC?

class Lidar
{
public:
    Lidar(uint16_t TxPin, uint16_t RxPin, int baudRate);
    ~Lidar();

    uint8_t data[2048];

    int receiveData();

private:
    Uart uart;
    bool isReceiving = false;
    uint sleepTime = 10;
};
