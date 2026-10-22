#pragma once

#include "freertos/FreeRTOS.h"
#include <cstdint>

/*
Constants of robot motion: geometry, straight moves, rotations and motion estimation.
Duty is PWM duty in x/255, the same scale as PowerMode.
*/
namespace config
{
    // ---------- Units ----------

    // Multiply degrees by it to get radians
    constexpr float degToRad = 3.14159265f / 180.0f;

    // ---------- Robot ----------

    // Motor numbers in MotorManager (addMotor)
    constexpr int leftMotor = 1;
    constexpr int rightMotor = 2;

    // Distance between wheels (m), converts wheel distance difference to heading
    constexpr float wheelTrack = 0.162f;

    // ---------- Straight moves (MotorManager::moveForDistance) ----------

    // Soft start: duty ramps from 0 to PowerMode in rampSteps steps, one step every rampStepDelay
    constexpr int rampSteps = 20;
    constexpr TickType_t rampStepDelay = pdMS_TO_TICKS(50); // 20 * 50 ms = 1 s

    // Straight move loop runs every tick, so wheel stops close to its target
    constexpr TickType_t moveLoopDelay = 1; // 1 tick = 10 ms (CONFIG_FREERTOS_HZ = 100)
    constexpr float moveDt = 0.010f;        // same period in seconds

    // Sync PID: duty difference between wheels from difference of their progress (m).
    // Wheel that is ahead is slowed down, the one behind sped up, so robot drives straight
    constexpr float Kp_sync = 3000.0f;   // 1 cm difference → 30 duty
    constexpr float Ki_sync = 500.0f;    // 1 cm held for 1 s → 5 duty more
    constexpr float Kd_sync = 50.0f;     // 1 cm/s difference → 0.5 duty, damps weaving
    constexpr float maxSyncDuty = 60.0f; // limit of sync PID output and of its integral part

    // Braking close to target: duty per meter of remaining distance
    constexpr float Kp_position = 5000.0f; // 2 cm left → 100 duty, at fast (220) braking starts ~4.4 cm before target
    // Lowest duty at which robot still drives, braking never goes below it so wheels do not stall before target
    constexpr float minDriveDuty = 100.0f; // tune on the robot

    // Move is stopped when it takes longer, robot is probably stuck
    constexpr float maxMoveTime = 20.0f; // seconds

    // ---------- Rotations (MotorManager::rotateForAngle) ----------

    // Rotation control loop runs after every estimate, so it is the same as estimation period
    constexpr float controlDt = 0.020f; // seconds

    // Smaller rotations are not worth starting motors for
    constexpr float minRotation = 0.5f * degToRad;

    // Heading PID (duty per rad of heading error), output sign gives rotation direction,
    // so overshoot is corrected by turning back
    constexpr float Kp_rotation = 300.0f;            // 30 deg error → ~157 duty
    constexpr float Ki_rotation = 200.0f;            // only close to target, pushes through friction in the last degrees
    constexpr float Kd_rotation = 30.0f;             // duty per rad/s from gyroscope, brakes harder when approaching fast
    constexpr float integralZone = 10.0f * degToRad; // integral is used only when error is smaller
    constexpr float maxIntegralDuty = 40.0f;         // limit of integral part of PID output

    // Rotation duty: limit ramps up from minRotationDuty to PowerMode in rotationRampTime.
    // Motors do not move below minRotationDuty, so smaller PID output is raised to it
    constexpr float minRotationDuty = 80.0f; // lowest duty at which robot still rotates, tune on the robot
    constexpr float rotationRampTime = 0.5f; // seconds

    // Rotation is finished when heading stays within headingTolerance and turn rate below
    // settledTurnRate for settledSamplesNeeded loops in a row
    constexpr float headingTolerance = 1.0f * degToRad;
    constexpr float settledTurnRate = 5.0f * degToRad; // rad/s
    constexpr int settledSamplesNeeded = 5;            // 5 * 20 ms = 0.1 s

    // Wheel balance: duty from sum of signed wheel distances (ticks). Pure rotation has sum 0,
    // otherwise robot also moves forward/backward, both wheels are pushed back against it
    constexpr float Kp_balance = 0.05f;     // ~400 ticks (1 cm) → 20 duty
    constexpr float maxBalanceDuty = 40.0f; // limit of balance duty

    // After motors stop, heading is still tracked while robot comes to rest
    constexpr int settleSamples = 10; // 10 * 20 ms = 0.2 s

    // Rotation is stopped when it takes longer, robot is probably stuck
    constexpr float maxRotationTime = 10.0f; // seconds

    // Gyroscope saturates at +-500 deg/s range, rotation log warns when rate gets close to it
    constexpr float gyroSaturationWarning = 450.0f * degToRad;

    // ---------- Motion estimation (MotionEstimator) ----------

    // Estimation task period. Kalman noise below is per step, other period needs retuning
    constexpr TickType_t estimationDelay = pdMS_TO_TICKS(20); // 50 Hz

    // Estimation task priority is higher than robot requests task (6), so estimates come on time
    constexpr UBaseType_t estimationTaskPriority = 7;
    constexpr uint32_t estimationTaskStackSize = 4096; // bytes

    // Heading is clockwise positive. IMU Z axis points up, so the gyroscope
    // reads counter-clockwise as positive and is negated (checked on the robot)
    constexpr float gyroDirection = -1.0f;

    // Heading Kalman filter noise: q – process noise per step, r – measurement noise, p – initial variance
    constexpr float q_angle = 0.000001f; // gyroscope integration per step (rad²)
    constexpr float q_gyroBias = 0.0f;   // bias not learned while moving
    constexpr float r_angle = 0.25f;     // encoder heading, wheel slip and diameter mismatch (rad²)
    constexpr float p_gyroBias = 0.0f;   // bias learned while robot stands still, not by the filter

    // Weight of one gyroscope sample in running average of gyroscope bias while robot stands still
    constexpr float gyroBiasAveraging = 0.1f;

    // Velocity Kalman filter noise: q – process noise per step, r – measurement noise, p – initial variance
    constexpr float q_velocity = 0.0001f;    // accelerometer integration and vibration per step (m/s)²
    constexpr float q_accelBias = 0.000001f; // accelerometer bias drift per step
    constexpr float r_velocity = 0.0004f;    // encoder velocity (m/s)²
    constexpr float p_accelBias = 0.01f;     // initial accelerometer bias variance, tilt (m/s²)²

    // Packet types of frames sent to computer
    constexpr uint8_t velocityPacketType = 0x56;
    constexpr uint8_t anglePacketType = 0x41;
}
