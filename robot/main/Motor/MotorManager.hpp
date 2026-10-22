#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include "driver/gpio.h"
#include "Kalman/MotionEstimator.hpp"
#include "Motor/Motor.hpp"
#include "Motor/Encoder.hpp"

/*
Drives robot with two motors: 1 – left, 2 – right.
Robot only moves straight (forward/backward) or rotates in place.

Straight moves use only encoders, every wheel has absolute target position
in encoder ticks since boot. A straight move shifts targets of both wheels by
the same number of ticks, each wheel drives toward its target and stops on
its own when it reaches it. Targets are moved from previous targets, not from
where wheels stopped, so overshoot of one move is taken back in the next one.
While driving straight:
  - duty ramps up at start and goes down close to target (not below minDriveDuty),
  - sync PID on progress difference between wheels slows down the wheel that is ahead,
    so robot drives straight and both wheels reach their targets together.

MotionEstimator is told which move is done (setMotion).
Rotations use its heading and turn rate, control loop runs after every estimate:
  - heading PID sets rotation duty, derivative is gyroscope turn rate, integral works only
    close to target; negative output turns back, so overshoot is corrected,
  - duty limit ramps up at start, output smaller than minRotationDuty is raised to it,
  - wheels turn in opposite directions, forward/backward drift of the robot is pushed back,
  - rotation is finished when heading stays within tolerance and robot almost stopped turning.
Target heading is kept across rotations, so error of one rotation and heading
drift of straight moves are corrected in the next rotation.
After rotation, wheel targets are set to where wheels stopped.

Both encoders must count up when robot drives forward.
*/
class MotorManager
{
public:
    MotorManager(MotionEstimator &motionEstimator);

    Motor operator[](const int motorNumber)
    {
        return motors.at(motorNumber);
    }

    void addMotor(const int motorNumber, Motor &motor);

    void moveForDistance(float distanceInMeters, PowerMode powerMode);
    void rotateForAngle(float angleInDegrees, PowerMode powerMode);

    void stop();

    // Prints every control loop sample of rotations (CSV lines starting with "rotlog")
    bool logRotation{false};

private:
    MotionEstimator &motionEstimator;
    std::map<int, Motor> motors;
    std::map<int, int64_t> targetTicks; // absolute wheel target positions (ticks since boot)
    float targetHeading{0.0f}; // heading robot should have, since boot (rad, clockwise positive)

    int64_t distanceToTicks(int motorNumber, float distanceInMeters);
    void driveToTargets(PowerMode powerMode);
};
