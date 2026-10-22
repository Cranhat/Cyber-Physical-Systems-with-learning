#include "Accelerometer.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <cstdint>
#include <stdio.h>

namespace
{
    constexpr uint8_t i2cAddress = 0x69;
    constexpr uint8_t cmdRegister = 0x7E;
    constexpr uint8_t dataRegister = 0x0C;
    constexpr int gyroscopeAxisCount = 3;
    constexpr int gyroscopeCalibrationSamples = 300;
    constexpr TickType_t gyroscopeCalibrationDelayTicks = pdMS_TO_TICKS(5);

    int16_t readInt16(const uint8_t *buffer, int index)
    {
        return static_cast<int16_t>((static_cast<uint16_t>(buffer[index + 1]) << 8) | buffer[index]);
    }

    void writeInt16(uint8_t *buffer, int index, int16_t value)
    {
        buffer[index] = value & 0xFF;
        buffer[index + 1] = (value >> 8) & 0xFF;
    }

    int16_t clampToInt16(int32_t value)
    {
        return static_cast<int16_t>(std::clamp<int32_t>(value, INT16_MIN, INT16_MAX));
    }
}

Accelerometer::Accelerometer(I2c &i2c) : i2c(i2c)
{
    data[0] = 0x67;
    initializeI2c();
}

Accelerometer::~Accelerometer()
{
    isMeasuring = false;
}

void Accelerometer::initializeI2c()
{
    slave = i2c.createSlave(i2cAddress);
    calibrate();
}

void Accelerometer::calibrate()
{
    constexpr uint8_t calibrateCommands[3] = {0xB6, 0x11, 0x15};
    constexpr uint8_t calibrateCommandsSize = 3;

    for (int i = 0; i < calibrateCommandsSize; i++)
    {
        uint8_t currentReg[2] = {cmdRegister, calibrateCommands[i]};
        i2c.transmit(slave, cmdRegister, currentReg, 2);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    uint8_t gyroscopeRange[2] = {0x43, 0x03};
    i2c.transmit(slave, gyroscopeRange[0], gyroscopeRange, 2);

    calibrateGyroscopeOffset();
}

void Accelerometer::calibrateGyroscopeOffset()
{
    std::array<int64_t, gyroscopeAxisCount> offsetSum{};

    for (int sample = 0; sample < gyroscopeCalibrationSamples; sample++)
    {
        i2c.transmitReceive(slave, dataRegister, receiveBuffer, 12);

        for (int axis = 0; axis < gyroscopeAxisCount; axis++)
        {
            offsetSum[axis] += readInt16(receiveBuffer, axis * 2);
        }

        vTaskDelay(gyroscopeCalibrationDelayTicks);
    }

    for (int axis = 0; axis < gyroscopeAxisCount; axis++)
    {
        gyroscopeOffset[axis] = static_cast<int16_t>(offsetSum[axis] / gyroscopeCalibrationSamples);
    }

    printf("Gyroscope offset: x=%d y=%d z=%d\n",
           gyroscopeOffset[0],
           gyroscopeOffset[1],
           gyroscopeOffset[2]);
}

int Accelerometer::receiveData()
{
    auto currentTimestamp = std::chrono::steady_clock::now();
    const std::chrono::duration elapsed{currentTimestamp - previousTimestamp};
    previousTimestamp = currentTimestamp;

    i2c.transmitReceive(slave, dataRegister, receiveBuffer, 12);
    for (int i = 1; i < 13; i++)
    {
        data[i] = receiveBuffer[i - 1];
    }

    for (int axis = 0; axis < gyroscopeAxisCount; axis++)
    {
        const int rawGyroscope = readInt16(receiveBuffer, axis * 2);
        const int16_t correctedGyroscope = clampToInt16(rawGyroscope - gyroscopeOffset[axis]);
        writeInt16(data, 1 + axis * 2, correctedGyroscope);
    }

    uint64_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();

    for (int i = 13; i < 13 + 8; i++)
    {
        data[i] = nano & 0xFF;
        nano >>= 8;
    }
    return 21;
}

// Packet Accelerometer::receivePackedData()
// {
//     packedData[0] = 0x67;
//     std::memcpy(packedData + 1, &data, sizeof(AccelerometerData));
//     return Packet(packedData, 21);
// }
