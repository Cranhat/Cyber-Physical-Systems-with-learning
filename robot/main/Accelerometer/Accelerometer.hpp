#pragma once

#include <driver/i2c_master.h>
#include <atomic>
#include "I2c/I2c.hpp"
#include <array>

/*
Reads gyroscope and accelerometer, offsets are calibrated at start.
*/

class Accelerometer
{
public:
    Accelerometer(I2c &i2c);
    ~Accelerometer();

    // Returns false when read failed, then previous values are given
    bool readMotion(float &gyroZ, float &accelForward);

private:
    void initializeI2c();
    void calibrate();
    void calibrateGyroscopeOffset();

    I2c &i2c;
    std::shared_ptr<I2cSlave> slave;

    bool isMeasuring = false;
    uint8_t receiveBuffer[12];
    std::array<int16_t, 3> gyroscopeOffset{};
    int16_t accelerationForwardOffset{0};
    float lastGyroZ{0.0f};
    float lastAccelForward{0.0f};
};
