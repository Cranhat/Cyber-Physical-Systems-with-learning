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
    class Ramp
    {
    public:
        explicit Ramp(PowerMode powerMode)
            : maxDuty(static_cast<float>(powerMode)), lastStepTime(xTaskGetTickCount())
        {
        }

        float duty()
        {
            if (step < rampSteps && xTaskGetTickCount() - lastStepTime >= rampStepDelay)
            {
                step++;
                lastStepTime = xTaskGetTickCount();
            }
            return maxDuty * static_cast<float>(std::max(step, 1)) / static_cast<float>(rampSteps);
        }

    private:
        const float maxDuty;
        int step{0};
        TickType_t lastStepTime;
    };
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

void MotorManager::cancel()
{
    cancelRequested.store(true);
}

void MotorManager::clearCancel()
{
    cancelRequested.store(false);
}

void MotorManager::setTargetsToCurrentPosition()
{
    for (auto &[motorNumber, motor] : motors)
    {
        targetTicks[motorNumber] = motor.getEncoderTicks();
    }
    targetHeading = motionEstimator.getHeading();
}

int64_t MotorManager::distanceToTicks(int motorNumber, float distanceInMeters)
{
    const Encoder &encoder = motors.at(motorNumber).encoder;
    return std::lround(distanceInMeters / encoder.getCorrectedDistanceInMeters(1));
}

bool MotorManager::moveForDistance(float distanceInMeters, PowerMode powerMode)
{
    targetTicks[leftMotor] += distanceToTicks(leftMotor, distanceInMeters);
    targetTicks[rightMotor] += distanceToTicks(rightMotor, distanceInMeters);
    return driveToTargets(powerMode);
}

bool MotorManager::rotateForAngle(float angleInDegrees, PowerMode powerMode)
{
    // Positive angle turns clockwise: left wheel forward, right wheel backward.
    targetHeading += angleInDegrees * degToRad;
    if (std::abs(targetHeading - motionEstimator.getHeading()) < minRotation)
    {
        return true;
    }

    motionEstimator.setMotion(Motion::rotating);
    const float maxDuty = static_cast<float>(powerMode);

    Motor &left = motors.at(leftMotor);
    Motor &right = motors.at(rightMotor);
    const int64_t leftStart = left.getEncoderTicks();
    const int64_t rightStart = right.getEncoderTicks();

    // Used only inside settleZone, so its integral does not grow during the long saturated part
    Pid headingPid{Kp_rotation, Ki_rotation, Kd_rotation, maxDuty, controlDt};
    bool settling = false;
    Ramp ramp{powerMode};
    int settledSamples = 0;
    float elapsed = 0.0f;
    bool cancelled = false;

    while (true)
    {
        if (cancelRequested.load())
        {
            printf("Rotation cancelled\n");
            cancelled = true;
            break;
        }

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

        // Rotation duty, positive turns clockwise. Inside tolerance motors are off, so robot settles.
        const float rampDuty = ramp.duty();
        float rotationDuty = 0.0f;
        if (std::abs(error) >= headingTolerance)
        {
            if (std::abs(error) < settleZone)
            {
                // Last degrees: heading PID, negative output turns back so overshoot is corrected.
                // Duty is never below minRotationDuty, so motors still move
                if (!settling)
                {
                    headingPid.reset();
                    settling = true;
                }
                const float pid = headingPid.update(error);
                const float dutyLimit = std::max(minRotationDuty, rampDuty);
                rotationDuty = std::copysign(std::clamp(std::abs(pid), minRotationDuty, dutyLimit), pid);
            }
            else
            {
                // Same as straight moves: ramp up, slow down close to target, not below minRotationDuty
                settling = false;
                const float brakeDuty = std::max(minRotationDuty, Kp_rotationBrake * std::abs(error));
                rotationDuty = std::copysign(std::min(rampDuty, brakeDuty), error);
            }
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
    if (cancelled)
    {
        setTargetsToCurrentPosition();
        return false;
    }
    targetTicks[leftMotor] = left.getEncoderTicks();
    targetTicks[rightMotor] = right.getEncoderTicks();
    return true;
}

bool MotorManager::driveToTargets(PowerMode powerMode)
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

    Ramp ramp{powerMode};
    float elapsed = 0.0f;
    TickType_t lastWakeTime = xTaskGetTickCount();
    bool cancelled = false;

    while (true)
    {
        if (cancelRequested.load())
        {
            printf("Move cancelled\n");
            stop();
            cancelled = true;
            break;
        }

        const float rampDuty = ramp.duty();

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
    if (cancelled)
    {
        setTargetsToCurrentPosition();
    }
    return !cancelled;
}
