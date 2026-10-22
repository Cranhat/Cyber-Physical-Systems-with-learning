#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include "driver/gpio.h"
#include "Motor/Motor.hpp"
#include "Motor/Encoder.hpp"
#include "Motor/Controller.hpp"

class MotorManager
{
public:
    Motor operator[](const int motorNumber)
    {
        return motors.at(motorNumber);
    }

    void addMotor(const int motorNumber, Motor &motor);

    // Move robot with given velocity (m/s) using EKF and PID closed loop with heading lock
    void moveWithVelocity(float velocityInMpS, float imuAccel = 0.0f, float imuGyroZ = 0.0f);
    void moveWithVelocity(float velocityInMpS, uint32_t durationMs, float imuAccel = 0.0f, float imuGyroZ = 0.0f);

    // Move robot for a specified distance (meters) using EKF and PID closed loop with heading lock
    void moveForDistance(float distanceInMeters, float velocityInMpS = 0.2f, float imuAccel = 0.0f, float imuGyroZ = 0.0f);
    void moveForDistance(float distanceInMeters, PowerMode powerMode, float imuAccel = 0.0f, float imuGyroZ = 0.0f);

    // Rotate robot for a specified angle (degrees) using EKF (gyro Z + encoders) and PID
    void rotateForAngle(float angleInDegrees, float speedMps = 0.2f, float imuGyroZ = 0.0f);
    void rotateForAngle(float angleInDegrees, PowerMode powerMode, float imuGyroZ = 0.0f);

    // Tick-based movement functions
    void moveMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuAccel = 0.0f, float imuGyroZ = 0.0f);
    void rotateMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuGyroZ = 0.0f);

    // Estimated robot pose from EKF
    float getX() const;
    float getY() const;
    float getYaw() const;        // in radians
    float getYawDegrees() const; // in degrees

    void stop();

private:
    std::map<int, Motor> motors;
    std::map<int, Controller> controllers;
    bool isDrivingStraight{false};
    float targetStraightHeading{0.0f};
};

