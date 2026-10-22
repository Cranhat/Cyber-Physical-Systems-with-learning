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
    constexpr float pi = 3.14159265f;

    // Heading-lock steering gain: (m/s) speed adjustment per radian of heading error
    constexpr float K_steer = 0.8f;
    // Maximum velocity adjustment applied to any wheel for steering (m/s)
    constexpr float maxSteerAdjustment = 0.12f;

    // Velocity setpoint derived from PowerMode duty [0-255] → m/s
    // Approximate: full duty (~255) ≈ 0.5 m/s for this robot
    constexpr float dutyToVelocity = 0.5f / 255.0f;

    int64_t absTicks(int64_t ticks)
    {
        return ticks < 0 ? -ticks : ticks;
    }

    float normalizeAngle(float angle)
    {
        while (angle > pi) angle -= 2.0f * pi;
        while (angle < -pi) angle += 2.0f * pi;
        return angle;
    }
}

void MotorManager::addMotor(const int motorNumber, Motor &motor)
{
    motors.insert({motorNumber, motor});
    controllers.emplace(motorNumber, motor.encoder);
}

void MotorManager::stop()
{
    isDrivingStraight = false;
    for (auto &[motorNumber, motor] : motors)
    {
        motor.stop();
        controllers.at(motorNumber).reset();
    }
}

float MotorManager::getX() const
{
    return controllers.empty() ? 0.0f : controllers.begin()->second.getEkf().getX();
}

float MotorManager::getY() const
{
    return controllers.empty() ? 0.0f : controllers.begin()->second.getEkf().getY();
}

float MotorManager::getYaw() const
{
    return controllers.empty() ? 0.0f : controllers.begin()->second.getEkf().getYaw();
}

float MotorManager::getYawDegrees() const
{
    return controllers.empty() ? 0.0f : controllers.begin()->second.getEkf().getYawDegrees();
}

void MotorManager::moveWithVelocity(float velocityInMpS, float imuAccel, float imuGyroZ)
{
    if (std::abs(velocityInMpS) < 0.001f)
    {
        stop();
        return;
    }

    // Lock target heading at the beginning of straight movement
    if (!isDrivingStraight)
    {
        targetStraightHeading = getYaw();
        isDrivingStraight = true;
    }

    const bool forward = velocityInMpS >= 0.0f;
    const float baseSpeed = std::abs(velocityInMpS);

    // Calculate heading error and steering correction
    const float headingError = normalizeAngle(targetStraightHeading - getYaw());
    const float steer = std::clamp(K_steer * headingError, -maxSteerAdjustment, maxSteerAdjustment);

    // Left (1) and Right (2) differential speeds to maintain heading
    const float vLeft  = std::max(0.0f, baseSpeed - (forward ? steer : -steer));
    const float vRight = std::max(0.0f, baseSpeed + (forward ? steer : -steer));

    // Approximate differential encoder yaw
    float encoderYaw = getYaw();
    if (motors.count(1) && motors.count(2))
    {
        const float vL = motors.at(1).encoder.getVelocityInMpS();
        const float vR = motors.at(2).encoder.getVelocityInMpS();
        encoderYaw += ((vR - vL) / wheelTrack) * controlDt;
    }

    if (controllers.count(1))
    {
        controllers.at(1).setTarget(vLeft);
        const uint8_t duty = controllers.at(1).tick(imuAccel, imuGyroZ, controlDt, encoderYaw);
        motors.at(1).setDuty(forward, duty);
    }
    if (controllers.count(2))
    {
        controllers.at(2).setTarget(vRight);
        const uint8_t duty = controllers.at(2).tick(imuAccel, imuGyroZ, controlDt, encoderYaw);
        motors.at(2).setDuty(forward, duty);
    }
}

void MotorManager::moveWithVelocity(float velocityInMpS, uint32_t durationMs, float imuAccel, float imuGyroZ)
{
    const uint32_t steps = durationMs / 20;
    for (uint32_t i = 0; i < steps; ++i)
    {
        moveWithVelocity(velocityInMpS, imuAccel, imuGyroZ);
        vTaskDelay(controlLoopDelay);
    }
    stop();
}

void MotorManager::moveForDistance(float distanceInMeters, float velocityInMpS, float imuAccel, float imuGyroZ)
{
    const float targetDistance = std::abs(distanceInMeters);
    const bool forward = distanceInMeters >= 0.0f;
    const float baseSpeed = std::abs(velocityInMpS);

    std::map<int, int64_t> startTicks;
    std::map<int, bool> done;

    for (auto &[motorNumber, motor] : motors)
    {
        startTicks[motorNumber] = motor.getEncoderTicks();
        done[motorNumber] = false;
        controllers.at(motorNumber).reset();
        controllers.at(motorNumber).setTarget(baseSpeed);
    }

    const float startX = getX();
    const float startY = getY();
    const float targetHeading = getYaw();

    while (true)
    {
        bool allDone = true;

        // Heading lock: steer back towards targetHeading
        const float headingError = normalizeAngle(targetHeading - getYaw());
        const float steer = std::clamp(K_steer * headingError, -maxSteerAdjustment, maxSteerAdjustment);
        const float vLeft  = std::max(0.0f, baseSpeed - (forward ? steer : -steer));
        const float vRight = std::max(0.0f, baseSpeed + (forward ? steer : -steer));

        if (controllers.count(1)) controllers.at(1).setTarget(vLeft);
        if (controllers.count(2)) controllers.at(2).setTarget(vRight);

        // Compute differential encoder yaw
        float movedLeft = 0.0f;
        float movedRight = 0.0f;
        if (motors.count(1))
        {
            const int64_t movedT = motors.at(1).getEncoderTicks() - startTicks.at(1);
            movedLeft = motors.at(1).encoder.getDistanceInMeters(movedT) * (forward ? 1.0f : -1.0f);
        }
        if (motors.count(2))
        {
            const int64_t movedT = motors.at(2).getEncoderTicks() - startTicks.at(2);
            movedRight = motors.at(2).encoder.getDistanceInMeters(movedT) * (forward ? 1.0f : -1.0f);
        }
        const float encoderYaw = targetHeading + (movedRight - movedLeft) / wheelTrack;

        for (auto &[motorNumber, motor] : motors)
        {
            if (done.at(motorNumber))
            {
                continue;
            }

            const int64_t movedTicks = motor.getEncoderTicks() - startTicks.at(motorNumber);
            const float movedDistance = std::abs(motor.encoder.getDistanceInMeters(movedTicks));

            // EKF 2D distance traveled
            const float dx = controllers.at(motorNumber).getEkf().getX() - startX;
            const float dy = controllers.at(motorNumber).getEkf().getY() - startY;
            const float ekfDistance = std::sqrt(dx * dx + dy * dy);

            if (ekfDistance >= targetDistance || movedDistance >= targetDistance * motor.encoder.tickCorrection)
            {
                motor.stop();
                done[motorNumber] = true;
            }
            else
            {
                const uint8_t duty = controllers.at(motorNumber).tick(imuAccel, imuGyroZ, controlDt, encoderYaw);
                motor.setDuty(forward, duty);
                allDone = false;
            }
        }

        if (allDone)
        {
            break;
        }

        vTaskDelay(controlLoopDelay);
    }
}

void MotorManager::moveForDistance(float distanceInMeters, PowerMode powerMode, float imuAccel, float imuGyroZ)
{
    moveForDistance(distanceInMeters, static_cast<float>(powerMode) * dutyToVelocity, imuAccel, imuGyroZ);
}

void MotorManager::moveMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuAccel, float imuGyroZ)
{
    const int64_t targetTicks = absTicks(ticks);
    const bool forward = ticks >= 0;
    const float baseSpeed = static_cast<float>(powerMode) * dutyToVelocity;

    std::map<int, int64_t> startTicks;
    std::map<int, bool> done;

    for (auto &[motorNumber, motor] : motors)
    {
        startTicks[motorNumber] = motor.getEncoderTicks();
        done[motorNumber] = false;
        controllers.at(motorNumber).reset();
        controllers.at(motorNumber).setTarget(baseSpeed);
    }

    const float targetHeading = getYaw();

    while (true)
    {
        bool allDone = true;

        // Heading lock steering
        const float headingError = normalizeAngle(targetHeading - getYaw());
        const float steer = std::clamp(K_steer * headingError, -maxSteerAdjustment, maxSteerAdjustment);
        const float vLeft  = std::max(0.0f, baseSpeed - (forward ? steer : -steer));
        const float vRight = std::max(0.0f, baseSpeed + (forward ? steer : -steer));

        if (controllers.count(1)) controllers.at(1).setTarget(vLeft);
        if (controllers.count(2)) controllers.at(2).setTarget(vRight);

        float movedLeft = 0.0f;
        float movedRight = 0.0f;
        if (motors.count(1))
        {
            const int64_t movedT = motors.at(1).getEncoderTicks() - startTicks.at(1);
            movedLeft = motors.at(1).encoder.getDistanceInMeters(movedT) * (forward ? 1.0f : -1.0f);
        }
        if (motors.count(2))
        {
            const int64_t movedT = motors.at(2).getEncoderTicks() - startTicks.at(2);
            movedRight = motors.at(2).encoder.getDistanceInMeters(movedT) * (forward ? 1.0f : -1.0f);
        }
        const float encoderYaw = targetHeading + (movedRight - movedLeft) / wheelTrack;

        for (auto &[motorNumber, motor] : motors)
        {
            if (done.at(motorNumber))
            {
                continue;
            }

            const int64_t movedTicks = motor.getEncoderTicks() - startTicks.at(motorNumber);
            if (absTicks(movedTicks) >= targetTicks * motor.encoder.tickCorrection)
            {
                motor.stop();
                done[motorNumber] = true;
            }
            else
            {
                const uint8_t duty = controllers.at(motorNumber).tick(imuAccel, imuGyroZ, controlDt, encoderYaw);
                motor.setDuty(forward, duty);
                allDone = false;
            }
        }

        if (allDone)
        {
            break;
        }

        vTaskDelay(controlLoopDelay);
    }
}

void MotorManager::rotateForAngle(float angleInDegrees, float speedMps, float imuGyroZ)
{
    if (std::abs(angleInDegrees) < 0.1f)
    {
        stop();
        return;
    }

    isDrivingStraight = false;
    const float targetAngleRad = std::abs(angleInDegrees) * degToRad;
    const bool leftForward = angleInDegrees >= 0.0f;
    const float targetVelocity = std::abs(speedMps);

    std::map<int, int64_t> startTicks;
    std::map<int, bool> directions;

    for (auto &[motorNumber, motor] : motors)
    {
        const bool forward = (motorNumber == 1) ? leftForward : !leftForward;
        startTicks[motorNumber] = motor.getEncoderTicks();
        directions[motorNumber] = forward;
        controllers.at(motorNumber).reset();
        controllers.at(motorNumber).setTarget(targetVelocity);
    }

    while (true)
    {
        float movedLeft = 0.0f;
        float movedRight = 0.0f;

        if (motors.count(1))
        {
            const int64_t ticks = motors.at(1).getEncoderTicks() - startTicks.at(1);
            movedLeft = motors.at(1).encoder.getDistanceInMeters(ticks) * (leftForward ? 1.0f : -1.0f);
        }
        if (motors.count(2))
        {
            const int64_t ticks = motors.at(2).getEncoderTicks() - startTicks.at(2);
            movedRight = motors.at(2).encoder.getDistanceInMeters(ticks) * (!leftForward ? 1.0f : -1.0f);
        }

        const float encoderYaw = (movedRight - movedLeft) / wheelTrack;

        for (auto &[motorNumber, motor] : motors)
        {
            const uint8_t duty = controllers.at(motorNumber).tick(0.0f, imuGyroZ, controlDt, encoderYaw);
            motor.setDuty(directions.at(motorNumber), duty);
        }

        // Check if fused yaw from EKF reached target angle
        const float fusedYaw = std::abs(controllers.begin()->second.getEkf().getYaw());
        if (fusedYaw >= targetAngleRad)
        {
            stop();
            break;
        }

        vTaskDelay(controlLoopDelay);
    }
}

void MotorManager::rotateForAngle(float angleInDegrees, PowerMode powerMode, float imuGyroZ)
{
    rotateForAngle(angleInDegrees, static_cast<float>(powerMode) * dutyToVelocity, imuGyroZ);
}

void MotorManager::rotateMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuGyroZ)
{
    isDrivingStraight = false;
    const int64_t targetTicks = absTicks(ticks);
    const bool leftForward = ticks >= 0;
    const float targetVelocity = static_cast<float>(powerMode) * dutyToVelocity;

    std::map<int, int64_t> startTicks;
    std::map<int, bool> directions;
    std::map<int, bool> done;

    for (auto &[motorNumber, motor] : motors)
    {
        const bool forward = motorNumber == 1 ? leftForward : !leftForward;
        startTicks[motorNumber] = motor.getEncoderTicks();
        directions[motorNumber] = forward;
        done[motorNumber] = false;
        controllers.at(motorNumber).reset();
        controllers.at(motorNumber).setTarget(targetVelocity);
    }

    while (true)
    {
        bool allDone = true;

        float movedLeft = 0.0f;
        float movedRight = 0.0f;
        if (motors.count(1))
        {
            const int64_t movedT = motors.at(1).getEncoderTicks() - startTicks.at(1);
            movedLeft = motors.at(1).encoder.getDistanceInMeters(movedT) * (leftForward ? 1.0f : -1.0f);
        }
        if (motors.count(2))
        {
            const int64_t movedT = motors.at(2).getEncoderTicks() - startTicks.at(2);
            movedRight = motors.at(2).encoder.getDistanceInMeters(movedT) * (!leftForward ? 1.0f : -1.0f);
        }
        const float encoderYaw = (movedRight - movedLeft) / wheelTrack;

        for (auto &[motorNumber, motor] : motors)
        {
            if (done.at(motorNumber))
            {
                continue;
            }

            const int64_t movedTicks = motor.getEncoderTicks() - startTicks.at(motorNumber);
            if (absTicks(movedTicks) >= targetTicks * motor.encoder.tickCorrection)
            {
                motor.stop();
                done[motorNumber] = true;
            }
            else
            {
                const uint8_t duty = controllers.at(motorNumber).tick(0.0f, imuGyroZ, controlDt, encoderYaw);
                motor.setDuty(directions.at(motorNumber), duty);
                allDone = false;
            }
        }

        if (allDone)
        {
            break;
        }

        vTaskDelay(controlLoopDelay);
    }
}
