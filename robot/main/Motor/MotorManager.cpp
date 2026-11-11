#include "MotorManager.hpp"

#include "Config.hpp"
#include "Motor/Pid.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <stdio.h>

using namespace config;

namespace
{
    uint8_t toDuty(float duty)
    {
        return static_cast<uint8_t>(std::clamp(duty, 0.0f, 255.0f));
    }
}

MotorManager::MotorManager(MotionEstimator &motionEstimator)
    : motionEstimator(motionEstimator)
{
}

void MotorManager::addMotor(const int motorNumber, Motor &motor)
{
    motors.insert({motorNumber, motor});
    targetTicks[motorNumber] = motor.getEncoderTicks();
}

void MotorManager::stop()
{
    for (auto &[motorNumber, motor] : motors)
    {
        motor.stop();
    }
}

int64_t MotorManager::distanceToTicks(int motorNumber, float distanceInMeters)
{
    const Encoder &encoder = motors.at(motorNumber).encoder;
    return std::lround(distanceInMeters / encoder.getCorrectedDistanceInMeters(1));
}

void MotorManager::moveForDistance(float distanceInMeters, PowerMode powerMode)
{
    targetTicks[leftMotor] += distanceToTicks(leftMotor, distanceInMeters);
    targetTicks[rightMotor] += distanceToTicks(rightMotor, distanceInMeters);
    driveToTargets(powerMode);
}

void MotorManager::rotateForAngle(float angleInDegrees, PowerMode powerMode)
{
    // Positive angle turns clockwise: left wheel forward, right wheel backward.
    targetHeading += angleInDegrees * degToRad;
    if (std::abs(targetHeading - motionEstimator.getHeading()) < minRotation)
    {
        return;
    }

    motionEstimator.setMotion(Motion::rotating);
    const float maxDuty = static_cast<float>(powerMode);

    Motor &left = motors.at(leftMotor);
    Motor &right = motors.at(rightMotor);
    const int64_t leftStart = left.getEncoderTicks();
    const int64_t rightStart = right.getEncoderTicks();

    Pid headingPid{Kp_rotation, Ki_rotation, Kd_rotation, maxDuty, controlDt};
    int settledSamples = 0;
    float elapsed = 0.0f;

    while (true)
    {
        motionEstimator.waitForEstimate();
        const float turnRate = motionEstimator.getTurnRate();
        const float error = targetHeading - motionEstimator.getHeading();

        if (std::abs(error) < headingTolerance && std::abs(turnRate) < settledTurnRate)
        {
            settledSamples++;
        }
        else
        {
            settledSamples = 0;
        }
        if (settledSamples >= settledSamplesNeeded)
        {
            break;
        }
        if (elapsed > maxRotationTime)
        {
            printf("Rotation timeout, robot stuck?\n");
            break;
        }

        const float pid = headingPid.update(error);

        // Rotation duty, positive turns clockwise. Inside tolerance motors are off, so robot settles.
        // Duty limit ramps up at start, duty is never below minRotationDuty so motors still move
        float rotationDuty = 0.0f;
        if (std::abs(error) >= headingTolerance)
        {
            const float dutyLimit = minRotationDuty + (maxDuty - minRotationDuty) * std::min(1.0f, elapsed / rotationRampTime);
            rotationDuty = std::copysign(std::clamp(std::abs(pid), minRotationDuty, dutyLimit), pid);
        }

        // Pure rotation keeps sum of signed wheel distances at 0
        const int64_t translation = (left.getEncoderTicks() - leftStart) + (right.getEncoderTicks() - rightStart);
        const float balanceDuty = rotationDuty == 0.0f
                                      ? 0.0f
                                      : std::clamp(Kp_balance * static_cast<float>(translation), -maxBalanceDuty, maxBalanceDuty);

        // Signed duty of each wheel, positive is forward
        const float leftDuty = rotationDuty - balanceDuty;
        const float rightDuty = -rotationDuty - balanceDuty;
        left.setDuty(leftDuty >= 0.0f, toDuty(std::abs(leftDuty)));
        right.setDuty(rightDuty >= 0.0f, toDuty(std::abs(rightDuty)));

        elapsed += controlDt;
    }

    stop();

    vTaskDelay(pdMS_TO_TICKS(200)); // wait until stationary
    motionEstimator.setMotion(Motion::still);
    targetTicks[leftMotor] = left.getEncoderTicks();
    targetTicks[rightMotor] = right.getEncoderTicks();

}

void MotorManager::driveToTargets(PowerMode powerMode)
{
    Motor &left = motors.at(leftMotor);
    Motor &right = motors.at(rightMotor);
    const int64_t leftStart = left.getEncoderTicks();
    const int64_t rightStart = right.getEncoderTicks();
    motionEstimator.setMotion(Motion::straight);

    std::map<int, bool> forward;
    std::map<int, bool> reached;
    for (auto &[motorNumber, motor] : motors)
    {
        const int64_t remaining = targetTicks.at(motorNumber) - motor.getEncoderTicks();
        forward[motorNumber] = remaining > 0;
        reached[motorNumber] = remaining == 0;
    }

    // Wheel that is ahead is slowed down, the one behind sped up,
    // so robot drives straight and both wheels reach their targets together
    Pid syncPid{Kp_sync, Ki_sync, Kd_sync, maxSyncDuty, moveDt};

    int step = 0;
    float elapsed = 0.0f;
    TickType_t lastRampTime = xTaskGetTickCount();
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        if (step < rampSteps && xTaskGetTickCount() - lastRampTime >= rampStepDelay)
        {
            step++;
            lastRampTime = xTaskGetTickCount();
        }
        const float rampDuty = static_cast<float>(static_cast<int>(powerMode) * std::max(step, 1) / rampSteps);

        // Wheel reached its target when it got to or past it in its direction
        bool allReached = true;
        float remainingSum = 0.0f;
        int movingCount = 0;
        for (auto &[motorNumber, motor] : motors)
        {
            if (reached.at(motorNumber))
            {
                continue;
            }
            const int64_t remaining = targetTicks.at(motorNumber) - motor.getEncoderTicks();
            if (forward.at(motorNumber) ? remaining <= 0 : remaining >= 0)
            {
                motor.stop();
                reached[motorNumber] = true;
            }
            else
            {
                remainingSum += std::abs(motor.encoder.getDistanceInMeters(remaining));
                movingCount++;
                allReached = false;
            }
        }

        if (allReached)
        {
            break;
        }
        if (elapsed > maxMoveTime)
        {
            printf("Move timeout, robot stuck?\n");
            stop();
            break;
        }

        // Slow down close to target, duty never goes below ramp or minDriveDuty
        const float brakeDuty = std::max(minDriveDuty, Kp_position * remainingSum / static_cast<float>(movingCount));
        const float baseDuty = std::min(rampDuty, brakeDuty);

        const float leftProgress = std::abs(left.encoder.getDistanceInMeters(left.getEncoderTicks() - leftStart));
        const float rightProgress = std::abs(right.encoder.getDistanceInMeters(right.getEncoderTicks() - rightStart));
        const float syncDuty = syncPid.update(leftProgress - rightProgress);

        if (!reached.at(leftMotor))
        {
            left.setDuty(forward.at(leftMotor), toDuty(baseDuty - syncDuty));
        }
        if (!reached.at(rightMotor))
        {
            right.setDuty(forward.at(rightMotor), toDuty(baseDuty + syncDuty));
        }

        elapsed += moveDt;
        vTaskDelayUntil(&lastWakeTime, moveLoopDelay);
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // wait until stationary
    motionEstimator.setMotion(Motion::still);
}
