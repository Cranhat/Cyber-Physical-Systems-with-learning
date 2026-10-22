#include "MotorManager.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <algorithm>
#include <stdio.h>

namespace
{
    constexpr float wheelTrack = 0.162f; // wheel track in meters
    constexpr float degToRad = 3.14159265f / 180.0f;

    constexpr int leftMotor = 1;
    constexpr int rightMotor = 2;

    // Soft start: duty ramps from 0 to PowerMode in rampSteps steps
    constexpr int rampSteps = 20;
    constexpr TickType_t rampStepDelay = pdMS_TO_TICKS(50); // 20 * 50 ms = 1 s

    // Straight move loop runs every tick, so wheel stops close to its target
    constexpr TickType_t moveLoopDelay = 1; // 1 tick = 10 ms (CONFIG_FREERTOS_HZ = 100)
    constexpr float moveDt = 0.010f;        // seconds

    // Sync PID: duty difference between wheels from difference of their progress (m)
    constexpr float Kp_sync = 3000.0f; // 1 cm difference → 30 duty
    constexpr float Ki_sync = 500.0f;  // 1 cm held for 1 s → 5 duty more
    constexpr float Kd_sync = 50.0f;   // 1 cm/s difference → 0.5 duty, damps weaving
    constexpr float maxSyncDuty = 60.0f;

    // Braking close to target: duty per meter of remaining distance,
    // never lower than minDriveDuty so wheels do not stall before target
    constexpr float Kp_position = 5000.0f; // 2 cm left → 100 duty, at fast (220) braking starts ~4.4 cm before target
    constexpr float minDriveDuty = 100.0f; // lowest duty at which robot still drives, tune on the robot

    // Move is stopped when it takes longer, robot is probably stuck
    constexpr float maxMoveTime = 20.0f; // seconds

    // Rotation control loop, heading filter uses real time between updates
    constexpr TickType_t controlLoopDelay = pdMS_TO_TICKS(20); // 50 Hz

    // Heading is clockwise positive. IMU Z axis points up, so the gyroscope
    // reads counter-clockwise as positive and is negated (checked on the robot)
    constexpr float gyroDirection = -1.0f;

    // Gyroscope saturates at +-500 deg/s range, rotation log warns when rate gets close to it
    constexpr float gyroSaturationWarning = 450.0f * degToRad;

    // Gyroscope bias calibration before each rotation, robot stands still
    constexpr int gyroBiasSamples = 15; // 15 * 20 ms = 0.3 s

    // Smaller rotations are not worth starting motors for
    constexpr float minRotation = 0.5f * degToRad;

    // Heading PID (duty per rad of heading error), output sign gives rotation direction,
    // so overshoot is corrected by turning back
    constexpr float Kp_rotation = 300.0f; // 30 deg error → ~157 duty
    constexpr float Ki_rotation = 200.0f; // only close to target, pushes through friction in the last degrees
    constexpr float Kd_rotation = 30.0f;  // duty per rad/s from gyroscope, brakes harder when approaching fast
    constexpr float integralZone = 10.0f * degToRad; // integral is used only when error is smaller
    constexpr float maxIntegralDuty = 40.0f;

    // Rotation duty: limit ramps up from minRotationDuty to PowerMode in rotationRampTime.
    // Motors do not move below minRotationDuty, so smaller PID output is raised to it
    constexpr float minRotationDuty = 80.0f; // lowest duty at which robot still rotates, tune on the robot
    constexpr float rotationRampTime = 0.5f; // seconds

    // Rotation is finished when heading stays within tolerance and robot almost stopped turning
    constexpr float headingTolerance = 1.0f * degToRad;
    constexpr float settledTurnRate = 5.0f * degToRad; // rad/s
    constexpr int settledSamplesNeeded = 5;            // 5 * 20 ms = 0.1 s

    // Wheel balance: duty from sum of signed wheel distances (ticks). Pure rotation has sum 0,
    // otherwise robot also moves forward/backward, both wheels are pushed back against it
    constexpr float Kp_balance = 0.05f; // ~400 ticks (1 cm) → 20 duty
    constexpr float maxBalanceDuty = 40.0f;

    // After motors stop, heading is still tracked while robot comes to rest
    constexpr int settleSamples = 10; // 10 * 20 ms = 0.2 s

    // Rotation is stopped when it takes longer, robot is probably stuck
    constexpr float maxRotationTime = 10.0f; // seconds

    uint8_t toDuty(float duty)
    {
        return static_cast<uint8_t>(std::clamp(duty, 0.0f, 255.0f));
    }
}

MotorManager::MotorManager(Accelerometer &accelerometer)
    : accelerometer(accelerometer)
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
    const float ticksPerMeter = 1.0f / encoder.getDistanceInMeters(1);
    return std::lround(distanceInMeters * ticksPerMeter * encoder.tickCorrection);
}

void MotorManager::moveForDistance(float distanceInMeters, PowerMode powerMode)
{
    targetTicks[leftMotor] += distanceToTicks(leftMotor, distanceInMeters);
    targetTicks[rightMotor] += distanceToTicks(rightMotor, distanceInMeters);
    driveToTargets(powerMode);
}

float MotorManager::wheelDistance(int motorNumber, int64_t startTicks)
{
    Motor &motor = motors.at(motorNumber);
    // Same correction as in distanceToTicks, so encoder heading matches straight moves
    return motor.encoder.getDistanceInMeters(motor.getEncoderTicks() - startTicks) / motor.encoder.tickCorrection;
}

void MotorManager::calibrateGyroBias()
{
    for (int sample = 0; sample < gyroBiasSamples; sample++)
    {
        float gyroZ = 0.0f;
        float accelForward = 0.0f;
        accelerometer.readMotion(gyroZ, accelForward);
        motionEstimator.calibrateGyroBias(gyroDirection * gyroZ);
        vTaskDelay(controlLoopDelay);
    }
}

float MotorManager::updateHeading(float startHeading, int64_t leftStart, int64_t rightStart, float &turnRate)
{
    float gyroZ = 0.0f;
    float accelForward = 0.0f;
    accelerometer.readMotion(gyroZ, accelForward);
    const float gyroRate = gyroDirection * gyroZ;
    turnRate = gyroRate - motionEstimator.getGyroBias();

    // Real time since previous update, loop can be late when other tasks run
    const int64_t now = esp_timer_get_time();
    headingDt = static_cast<float>(now - lastHeadingUpdateUs) / 1000000.0f;
    lastHeadingUpdateUs = now;

    // Clockwise: left wheel forward (+), right wheel backward (-)
    const float encoderHeading = startHeading + (wheelDistance(leftMotor, leftStart) - wheelDistance(rightMotor, rightStart)) / wheelTrack;
    return motionEstimator.updateHeading(gyroRate, encoderHeading, headingDt);
}

void MotorManager::rotateForAngle(float angleInDegrees, PowerMode powerMode)
{
    // Positive angle turns clockwise: left wheel forward, right wheel backward.
    // Rotation goes to the new target heading, so error left from previous rotations is corrected too.
    targetHeading += angleInDegrees * degToRad;
    if (std::abs(targetHeading - motionEstimator.getAngle()) < minRotation)
    {
        return;
    }

    calibrateGyroBias();
    motionEstimator.reset();

    const float startHeading = motionEstimator.getAngle();
    const float maxDuty = static_cast<float>(powerMode);

    Motor &left = motors.at(leftMotor);
    Motor &right = motors.at(rightMotor);
    const int64_t leftStart = left.getEncoderTicks();
    const int64_t rightStart = right.getEncoderTicks();

    float integral = 0.0f;
    float previousError = targetHeading - startHeading;
    int settledSamples = 0;
    float elapsed = 0.0f;

    // Diagnostics: heading from gyroscope only and highest turn rate
    float gyroOnlyHeading = startHeading;
    float maxTurnRate = 0.0f;

    lastHeadingUpdateUs = esp_timer_get_time();
    TickType_t lastWakeTime = xTaskGetTickCount();

    if (logRotation)
    {
        printf("rotlog,phase,t_s,dt_ms,target_deg,heading_deg,gyro_only_deg,encoder_deg,error_deg,rate_dps,pid,duty_left,duty_right,ticks_left,ticks_right\n");
    }

    while (true)
    {
        float turnRate = 0.0f;
        const float heading = updateHeading(startHeading, leftStart, rightStart, turnRate);
        const float error = targetHeading - heading;
        gyroOnlyHeading += turnRate * headingDt;
        maxTurnRate = std::max(maxTurnRate, std::abs(turnRate));

        // Finished when robot stays at target heading and almost stopped turning
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

        // Integral only close to target, cleared when target is crossed so it does not push over it
        if ((error > 0.0f) != (previousError > 0.0f))
        {
            integral = 0.0f;
        }
        if (std::abs(error) < integralZone)
        {
            integral = std::clamp(integral + error * headingDt, -maxIntegralDuty / Ki_rotation, maxIntegralDuty / Ki_rotation);
        }
        previousError = error;

        // Derivative from gyroscope (derivative of error is -turnRate), no noise from differentiating heading
        const float pid = Kp_rotation * error + Ki_rotation * integral - Kd_rotation * turnRate;

        // Rotation duty, positive turns clockwise. Inside tolerance motors are off, so robot settles
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

        if (logRotation)
        {
            const float encoderHeading = startHeading + (wheelDistance(leftMotor, leftStart) - wheelDistance(rightMotor, rightStart)) / wheelTrack;
            printf("rotlog,run,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,%.0f,%lld,%lld\n",
                   elapsed, headingDt * 1000.0f, targetHeading / degToRad, heading / degToRad,
                   gyroOnlyHeading / degToRad, encoderHeading / degToRad, error / degToRad, turnRate / degToRad,
                   pid, leftDuty, rightDuty,
                   static_cast<long long>(left.getEncoderTicks() - leftStart),
                   static_cast<long long>(right.getEncoderTicks() - rightStart));
        }

        elapsed += headingDt;
        vTaskDelayUntil(&lastWakeTime, controlLoopDelay);
    }

    stop();

    // Heading is still tracked while robot comes to rest
    float heading = motionEstimator.getAngle();
    TickType_t settleWakeTime = xTaskGetTickCount();
    for (int sample = 0; sample < settleSamples; sample++)
    {
        float turnRate = 0.0f;
        heading = updateHeading(startHeading, leftStart, rightStart, turnRate);
        gyroOnlyHeading += turnRate * headingDt;
        if (logRotation)
        {
            const float encoderHeading = startHeading + (wheelDistance(leftMotor, leftStart) - wheelDistance(rightMotor, rightStart)) / wheelTrack;
            printf("rotlog,settle,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,0,0,0,%lld,%lld\n",
                   elapsed + (sample + 1) * 0.02f, headingDt * 1000.0f, targetHeading / degToRad, heading / degToRad,
                   gyroOnlyHeading / degToRad, encoderHeading / degToRad, (targetHeading - heading) / degToRad,
                   turnRate / degToRad,
                   static_cast<long long>(left.getEncoderTicks() - leftStart),
                   static_cast<long long>(right.getEncoderTicks() - rightStart));
        }
        vTaskDelayUntil(&settleWakeTime, controlLoopDelay);
    }

    // Straight moves continue from where wheels stopped
    targetTicks[leftMotor] = left.getEncoderTicks();
    targetTicks[rightMotor] = right.getEncoderTicks();

    const float encoderHeading = startHeading + (wheelDistance(leftMotor, leftStart) - wheelDistance(rightMotor, rightStart)) / wheelTrack;
    printf("Rotation: %.2f s, target %.1f deg, heading %.1f deg (gyro only %.1f, encoders %.1f deg), error %.1f deg, max rate %.0f deg/s\n",
           elapsed, targetHeading / degToRad, heading / degToRad, gyroOnlyHeading / degToRad, encoderHeading / degToRad,
           (heading - targetHeading) / degToRad, maxTurnRate / degToRad);
    if (maxTurnRate > gyroSaturationWarning)
    {
        printf("Warning: turn rate close to gyroscope range (+-500 deg/s), heading is too small, use slower PowerMode\n");
    }
}

void MotorManager::driveToTargets(PowerMode powerMode)
{
    Motor &left = motors.at(leftMotor);
    Motor &right = motors.at(rightMotor);
    const int64_t leftStart = left.getEncoderTicks();
    const int64_t rightStart = right.getEncoderTicks();

    std::map<int, bool> forward;
    std::map<int, bool> reached;
    for (auto &[motorNumber, motor] : motors)
    {
        const int64_t remaining = targetTicks.at(motorNumber) - motor.getEncoderTicks();
        forward[motorNumber] = remaining > 0;
        reached[motorNumber] = remaining == 0;
    }

    // Sync PID state
    float syncIntegral = 0.0f;
    float previousSyncError = 0.0f;

    int step = 0;
    float elapsed = 0.0f;
    TickType_t lastRampTime = xTaskGetTickCount();
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        // Ramp up duty, wheels are checked during ramp too, so short moves do not overshoot
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

        // Sync PID: wheel that is ahead is slowed down, the one behind sped up,
        // so robot drives straight and both wheels reach their targets together
        const float leftProgress = std::abs(left.encoder.getDistanceInMeters(left.getEncoderTicks() - leftStart));
        const float rightProgress = std::abs(right.encoder.getDistanceInMeters(right.getEncoderTicks() - rightStart));
        const float syncError = leftProgress - rightProgress;
        syncIntegral = std::clamp(syncIntegral + syncError * moveDt, -maxSyncDuty / Ki_sync, maxSyncDuty / Ki_sync);
        const float syncDerivative = (syncError - previousSyncError) / moveDt;
        previousSyncError = syncError;
        const float syncDuty = std::clamp(Kp_sync * syncError + Ki_sync * syncIntegral + Kd_sync * syncDerivative,
                                          -maxSyncDuty, maxSyncDuty);

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

    // Wheels coast a bit after stop, overshoot is taken back by the next move
    vTaskDelay(pdMS_TO_TICKS(200));
    for (auto &[motorNumber, motor] : motors)
    {
        printf("Motor %d: target %lld, position %lld, error %lld ticks\n", motorNumber,
               static_cast<long long>(targetTicks.at(motorNumber)),
               static_cast<long long>(motor.getEncoderTicks()),
               static_cast<long long>(motor.getEncoderTicks() - targetTicks.at(motorNumber)));
    }
}
