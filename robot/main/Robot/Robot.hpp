#pragma once

#include <atomic>
#include <cstdint>
#include "Accelerometer/Accelerometer.hpp"
#include "DataExchanger/DataExchanger.hpp"
#include "Led/Leds.hpp"
#include "Lidar/Lidar.hpp"
#include "Motor/MotorManager.hpp"

enum State : uint32_t
{
    Idle,
    Calibrating,
    Ready,
    Processing,
    Failure
};
static_assert(sizeof(State) == 4);

/*
Class of robot itself.
*/
class Robot
{
public:
    Robot(DataExchanger &dataExchanger, Accelerometer &accelerometer, Leds &leds, Lidar &lidar, MotorManager &motorManager);
    ~Robot();

    DataExchanger &dataExchanger;
    void moveForDistance(float distanceInMeters, PowerMode powerMode = slow);
    void rotateForAngle(float angleInDegrees, PowerMode powerMode = slow);
    void stopMotors();

    // State is atomic
    void setState(State newState);
    State getState() const;

private:
    Accelerometer &accelerometer;
    Leds &leds;
    Lidar &lidar;
    MotorManager &motorManager;

    std::atomic<State> state{Calibrating};

    void start();
    void stop();
};
