#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include "driver/gpio.h"
#include "Accelerometer/Accelerometer.hpp"
#include "Kalman/MotionEstimator.hpp"
#include "Motor/Motor.hpp"
#include "Motor/Encoder.hpp"
#include "Motor/Controller.hpp"

/*
Drives robot with two motors: 1 – left, 2 – right.
Robot only moves straight (forward/backward) or rotates in place.

Heading is kept across moves: straight moves hold targetHeading and
rotations turn it by the commanded angle, so errors do not add up.

During a move:
  - distance is measured from encoders since the start of the current move,
  - heading and velocity are estimated by MotionEstimator (IMU + encoders),
  - speed target ramps up at start, so both wheels start smoothly,
  - speed Controller sets common duty (feedforward + PID) from estimated velocity,
  - heading error is corrected by PID adding duty difference between wheels,
  - move stops early by distance the robot still travels while braking.
*/
class MotorManager
{
public:
    MotorManager(Accelerometer &accelerometer);

    Motor operator[](const int motorNumber)
    {
        return motors.at(motorNumber);
    }

    void addMotor(const int motorNumber, Motor &motor);

    // Move straight for a specified distance (meters, negative = backward)
    void moveForDistance(float distanceInMeters, PowerMode powerMode);

    // Rotate in place for a specified angle (degrees, positive = left wheel forward, clockwise)
    void rotateForAngle(float angleInDegrees, PowerMode powerMode);

    void stop();

private:
    Accelerometer &accelerometer;
    std::map<int, Motor> motors;
    std::map<int, int64_t> startTicks;
    MotionEstimator motionEstimator;
    Controller speedController;

    float targetHeading{0.0f};    // heading robot should have, since boot (rad, counter-clockwise positive)
    float moveStartHeading{0.0f}; // estimated heading at start of the current move (rad)
    float leftDirection{1.0f};    // wheel directions of the current move: 1 forward, -1 backward
    float rightDirection{1.0f};

    void startMove(bool leftForward, bool rightForward);
    void finishMove();
    float movedDistance(int motorNumber); // meters since start of the current move
    float updateHeading(float gyroZ);     // returns estimated heading (rad)
    float headingRate(float gyroZ) const; // bias corrected angular velocity (rad/s)
};
