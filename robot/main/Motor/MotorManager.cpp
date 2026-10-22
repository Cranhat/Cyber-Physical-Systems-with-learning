#include "MotorManager.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>

namespace
{
    constexpr TickType_t controlLoopDelay = pdMS_TO_TICKS(20); // 50 Hz control loop
    constexpr float controlDt = 0.020f;                        // seconds
    constexpr float wheelTrack = 0.145f;                       // wheel track in meters
    constexpr float degToRad = 3.14159265f / 180.0f;

    constexpr int leftMotor = 1;
    constexpr int rightMotor = 2;

    // IMU Z axis points up, so counter-clockwise rotation is positive,
    // same as encoder heading. Set to -1.0f if IMU is mounted upside down.
    constexpr float gyroDirection = 1.0f;

    // Heading PID: PWM duty difference per radian of heading error
    constexpr float Kp_heading = 300.0f;
    constexpr float Ki_heading = 100.0f;
    // Damping from gyroscope angular velocity (duty per rad/s), stops weaving
    constexpr float Kd_heading = 20.0f;
    // Maximum duty difference applied to any wheel for steering
    constexpr float maxSteerDuty = 60.0f;

    // Speed PID never uses full duty, so steering always has room
    constexpr float maxSpeedDuty = 255.0f - maxSteerDuty;

    // Soft start: speed target ramps from startSpeedRatio of target speed to full speed
    constexpr float startSpeedRatio = 0.4f;
    constexpr float speedRampTime = 0.3f; // seconds

    // Smaller rotations are not worth starting motors for
    constexpr float minRotation = 0.5f * degToRad;

    // After motors stop, heading is still tracked while robot comes to rest
    constexpr int settleSamples = 5; // 5 * 20 ms = 0.1 s

    // Gyroscope bias calibration before each move, robot stands still
    constexpr int gyroBiasSamples = 15; // 15 * 20 ms = 0.3 s

    // Time robot still moves after motors stop (s), tune by measuring overshoot
    constexpr float stopLookahead = 0.05f;

    // Velocity setpoint derived from PowerMode duty [0-255] → m/s
    // Approximate: full duty (~255) ≈ 0.5 m/s for this robot
    constexpr float dutyToVelocity = 0.5f / 255.0f;

    uint8_t toDuty(float duty)
    {
        return static_cast<uint8_t>(std::clamp(duty, 0.0f, 255.0f));
    }

    float rampedSpeed(float targetSpeed, float elapsed)
    {
        const float ratio = startSpeedRatio + (1.0f - startSpeedRatio) * elapsed / speedRampTime;
        return targetSpeed * std::min(1.0f, ratio);
    }
}

MotorManager::MotorManager(Accelerometer &accelerometer)
    : accelerometer(accelerometer),
      speedController(maxSpeedDuty, 1.0f / dutyToVelocity)
{
}

void MotorManager::addMotor(const int motorNumber, Motor &motor)
{
    motors.insert({motorNumber, motor});
}

void MotorManager::stop()
{
    for (auto &[motorNumber, motor] : motors)
    {
        motor.stop();
    }
    speedController.reset();
}

void MotorManager::startMove(bool leftForward, bool rightForward)
{
    for (int sample = 0; sample < gyroBiasSamples; sample++)
    {
        float gyroZ = 0.0f;
        float accelForward = 0.0f;
        accelerometer.readMotion(gyroZ, accelForward);
        motionEstimator.calibrateGyroBias(gyroDirection * gyroZ);
        vTaskDelay(controlLoopDelay);
    }

    motionEstimator.reset();
    speedController.reset();
    moveStartHeading = motionEstimator.getAngle();
    leftDirection = leftForward ? 1.0f : -1.0f;
    rightDirection = rightForward ? 1.0f : -1.0f;
    for (auto &[motorNumber, motor] : motors)
    {
        startTicks[motorNumber] = motor.getEncoderTicks();
    }
}

void MotorManager::finishMove()
{
    stop();

    // Robot still rolls a bit, keep tracking heading until it stands still
    TickType_t lastWakeTime = xTaskGetTickCount();
    for (int sample = 0; sample < settleSamples; sample++)
    {
        float gyroZ = 0.0f;
        float accelForward = 0.0f;
        accelerometer.readMotion(gyroZ, accelForward);
        updateHeading(gyroZ);
        vTaskDelayUntil(&lastWakeTime, controlLoopDelay);
    }
}

float MotorManager::movedDistance(int motorNumber)
{
    Motor &motor = motors.at(motorNumber);
    const int64_t movedTicks = motor.getEncoderTicks() - startTicks.at(motorNumber);
    return std::abs(motor.encoder.getDistanceInMeters(movedTicks));
}

float MotorManager::updateHeading(float gyroZ)
{
    // Encoder heading: heading at move start + wheel difference since then (counter-clockwise positive)
    const float movedLeft = leftDirection * movedDistance(leftMotor);
    const float movedRight = rightDirection * movedDistance(rightMotor);
    const float encoderHeading = moveStartHeading + (movedRight - movedLeft) / wheelTrack;
    return motionEstimator.updateHeading(gyroDirection * gyroZ, encoderHeading, controlDt);
}

float MotorManager::headingRate(float gyroZ) const
{
    return gyroDirection * gyroZ - motionEstimator.getGyroBias();
}

void MotorManager::moveForDistance(float distanceInMeters, PowerMode powerMode)
{
    const bool forward = distanceInMeters >= 0.0f;
    const float direction = forward ? 1.0f : -1.0f;
    const float targetDistance = std::abs(distanceInMeters) * motors.at(leftMotor).encoder.tickCorrection;
    const float targetSpeed = static_cast<float>(powerMode) * dutyToVelocity;

    startMove(forward, forward);

    float previousLeft = 0.0f;
    float previousRight = 0.0f;
    float headingIntegral = 0.0f;
    float elapsed = 0.0f;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        float gyroZ = 0.0f;
        float accelForward = 0.0f;
        accelerometer.readMotion(gyroZ, accelForward);

        const float heading = updateHeading(gyroZ);

        // Velocity along the direction of travel
        const float movedLeft = movedDistance(leftMotor);
        const float movedRight = movedDistance(rightMotor);
        const float encoderVelocity = ((movedLeft - previousLeft) + (movedRight - previousRight)) / 2.0f / controlDt;
        const float velocity = motionEstimator.updateVelocity(direction * accelForward, encoderVelocity, controlDt);
        previousLeft = movedLeft;
        previousRight = movedRight;

        // Stop early by distance traveled while braking
        const float distance = (movedLeft + movedRight) / 2.0f;
        if (distance + std::max(0.0f, velocity) * stopLookahead >= targetDistance)
        {
            break;
        }

        // Heading PID: positive error while driving forward means robot turned left (right wheel ahead),
        // so slow down right and speed up left. Driving backward it is mirrored.
        const float headingError = heading - targetHeading;
        headingIntegral += headingError * controlDt;
        const float steer = Kp_heading * headingError + Ki_heading * headingIntegral + Kd_heading * headingRate(gyroZ);
        const float steerDuty = direction * std::clamp(steer, -maxSteerDuty, maxSteerDuty);

        speedController.setTarget(rampedSpeed(targetSpeed, elapsed));
        elapsed += controlDt;

        const float baseDuty = speedController.tick(velocity);
        motors.at(leftMotor).setDuty(forward, toDuty(baseDuty + steerDuty));
        motors.at(rightMotor).setDuty(forward, toDuty(baseDuty - steerDuty));

        vTaskDelayUntil(&lastWakeTime, controlLoopDelay);
    }

    finishMove();
}

void MotorManager::rotateForAngle(float angleInDegrees, PowerMode powerMode)
{
    // Positive angle turns clockwise (left wheel forward), heading is counter-clockwise positive.
    // Rotation goes to the new target heading, so error left from previous moves is corrected too.
    targetHeading -= angleInDegrees * degToRad;
    const float remainingAngle = targetHeading - motionEstimator.getAngle();
    if (std::abs(remainingAngle) < minRotation)
    {
        return;
    }

    const bool counterClockwise = remainingAngle > 0.0f;
    const float rotationDirection = counterClockwise ? 1.0f : -1.0f;
    const bool leftForward = !counterClockwise;
    const float targetSpeed = static_cast<float>(powerMode) * dutyToVelocity;

    // Velocity filter is not updated while rotating: centripetal acceleration
    // of off-center IMU would corrupt accelerometer bias
    startMove(leftForward, !leftForward);

    float elapsed = 0.0f;
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        float gyroZ = 0.0f;
        float accelForward = 0.0f;
        accelerometer.readMotion(gyroZ, accelForward);

        const float heading = updateHeading(gyroZ);
        const float turnRate = std::abs(headingRate(gyroZ));

        // Stop early by angle turned while braking
        if (rotationDirection * (targetHeading - heading) <= turnRate * stopLookahead)
        {
            break;
        }

        speedController.setTarget(rampedSpeed(targetSpeed, elapsed));
        elapsed += controlDt;

        // Wheel speed along its arc
        const uint8_t duty = speedController.tick(turnRate * wheelTrack / 2.0f);
        motors.at(leftMotor).setDuty(leftForward, duty);
        motors.at(rightMotor).setDuty(!leftForward, duty);

        vTaskDelayUntil(&lastWakeTime, controlLoopDelay);
    }

    finishMove();
}
