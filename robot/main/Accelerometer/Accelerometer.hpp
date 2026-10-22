#pragma once

#include <driver/i2c_master.h>
#include <atomic>
#include "I2c/I2c.hpp"
#include <array>
#include <chrono>

/*
Gets data from accelerometer and converts it into packets,
which can be sent further. Packages are needed because this
is raw data afaik.
*/

// struct data {
//     uint8_t header{0x67};
//     int16_t gyroscopeX;
//     int16_t gyroscopeY;
//     int16_t gyroscopeZ;
//     int16_t accelerationX;
//     int16_t accelerationY;
//     int16_t accelerationZ;
// };
class Accelerometer
{
public:
    Accelerometer(I2c &i2c);
    ~Accelerometer();

    uint8_t data[21];

    int receiveData();
    // Reads gyroscope and accelerometer in one transfer, offset corrected:
    //   gyroZ        – angular velocity around Z axis in rad/s
    //   accelForward – acceleration along robot forward axis in m/s²
    void readMotion(float &gyroZ, float &accelForward);

private:
    I2c &i2c;
    bool isMeasuring = false;
    uint8_t receiveBuffer[12];
    std::array<int16_t, 3> gyroscopeOffset{};
    int16_t accelerationForwardOffset{0};
    std::shared_ptr<I2cSlave> slave;
    std::chrono::steady_clock::time_point previousTimestamp{
        std::chrono::steady_clock::now()};
    void initializeI2c();
    void calibrate();
    void calibrateGyroscopeOffset();
};
