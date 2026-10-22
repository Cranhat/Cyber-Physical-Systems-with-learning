#pragma once

#include <atomic>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Accelerometer/Accelerometer.hpp"
#include "Kalman/KalmanFilter.hpp"
#include "Motor/Encoder.hpp"

// Move robot is doing, set by MotorManager
enum class Motion
{
    still,
    straight,
    rotating
};

/*
Estimates robot motion with two independent Kalman filters. Own task (50 Hz)
reads IMU and encoders and updates both filters.
Robot only drives straight or rotates in place, so heading and velocity
are not coupled and each has its own 2-state filter.
Filters are reset at start of every move.

Heading filter:
  imuRate            – gyroscope Z (rad/s)
  encoderMeasurement – encoder heading, follows estimate while robot stands still,
                       then (distanceLeft - distanceRight) / wheelTrack is added (rad)
  estimate           – heading since boot, kept across moves (rad, clockwise positive)
  sensorBias         – gyroscope bias (rad/s)
Gyroscope is trusted for heading, encoders only slowly correct its drift.
Gyroscope bias is not learned while moving: wheel diameter mismatch looks
exactly like gyroscope bias there. It is learned only while robot stands
still, when true angular velocity is 0.

Velocity filter (accelerometer is not used while rotating, centripetal
acceleration of off-center IMU would corrupt accelerometer bias):
  imuRate            – accelerometer along direction of travel (m/s²)
  encoderMeasurement – average wheel velocity (m/s)
  estimate           – velocity along direction of travel, from 0 each move (m/s)
  sensorBias         – accelerometer bias (m/s²)

Sent to computer (packets [TYPE][PAYLOAD], little-endian):
  VELOCITY 0x56 – velocity as float (m/s), estimate time as uint64 (us since boot)
  ANGLE    0x41 – heading as float (rad, clockwise positive), estimate time as uint64 (us since boot)
*/
class MotionEstimator
{
public:
    MotionEstimator(Accelerometer &accelerometer, Encoder &leftEncoder, Encoder &rightEncoder);

    // Estimate from before is dropped, so next waitForEstimate gives one with new motion
    void setMotion(Motion newMotion);
    void waitForEstimate();

    float getHeading() const;
    float getTurnRate() const; // bias corrected gyroscope (rad/s)
    float getVelocity() const;

    int sendVelocity();
    int sendAngle();

    static constexpr int frameSize = 1 + 4 + 8;
    uint8_t velocityData[frameSize];
    uint8_t angleData[frameSize];

private:
    Accelerometer &accelerometer;
    Encoder &leftEncoder;
    Encoder &rightEncoder;
    KalmanFilter headingFilter;
    KalmanFilter velocityFilter;
    std::atomic<Motion> motion{Motion::still};

    std::atomic<float> heading{0.0f};
    std::atomic<float> turnRate{0.0f};
    std::atomic<float> velocity{0.0f};
    std::atomic<int64_t> estimateTimeUs{0};
    SemaphoreHandle_t newEstimate; // given after every estimate

    static void task(void *arg);
    void run();
};
