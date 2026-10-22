#pragma once

#include <cstdint>
#include "Accelerometer/Accelerometer.hpp"
#include "DataExchanger/DataExchanger.hpp"
#include "Led/Leds.hpp"
#include "Lidar/Lidar.hpp"
#include "Motor/MotorManager.hpp"

enum State
{
    stopped,
    connected,
    running,
    crashed
};

/*
Class of robot itself.
*/
class Robot
{
public:
    Robot(DataExchanger &dataExchanger, Accelerometer &accelerometer, Leds &leds, Lidar &lidar, MotorManager &motorManager);
    ~Robot();

    DataExchanger &dataExchanger;
    void moveMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuAccel = 0.0f, float imuGyroZ = 0.0f);
    void rotateMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuGyroZ = 0.0f);
    void moveWithVelocity(float velocityInMpS, float imuAccel = 0.0f, float imuGyroZ = 0.0f);
    void moveForDistance(float distanceInMeters, float velocityInMpS = 0.2f, float imuAccel = 0.0f, float imuGyroZ = 0.0f);
    void rotateForAngle(float angleInDegrees, float speedMps = 0.2f, float imuGyroZ = 0.0f);
    void stopMotors();

    // 2D Pose estimated by EKF
    float getX() const { return motorManager.getX(); }
    float getY() const { return motorManager.getY(); }
    float getYaw() const { return motorManager.getYaw(); }
    float getYawDegrees() const { return motorManager.getYawDegrees(); }

private:
    Accelerometer &accelerometer;
    Leds &leds;
    Lidar &lidar;
    MotorManager &motorManager;

    State state = stopped;

    void start();
    void stop();
};
