#pragma once

#include "Kalman/KalmanFilter.hpp"

/*
Estimates robot motion during a single move with two independent Kalman filters.
Robot only drives straight or rotates in place, so heading and velocity
are not coupled and each has its own 2-state filter.

Heading filter:
  imuRate            – gyroscope Z (rad/s)
  encoderMeasurement – heading at move start + (distanceRight - distanceLeft) / wheelTrack (rad)
  estimate           – heading since boot, kept across moves (rad, counter-clockwise positive)
  sensorBias         – gyroscope bias (rad/s)
Gyroscope is trusted for heading, encoders only slowly correct its drift.
Gyroscope bias is not learned while moving: wheel diameter mismatch looks
exactly like gyroscope bias there. It is learned only while robot stands
still (calibrateGyroBias), when true angular velocity is 0.

Velocity filter (straight moves only, centripetal acceleration
of off-center IMU would corrupt accelerometer bias while rotating):
  imuRate            – accelerometer along direction of travel (m/s²)
  encoderMeasurement – average wheel velocity (m/s)
  estimate           – velocity along direction of travel, from 0 each move (m/s)
  sensorBias         – accelerometer bias (m/s²)
*/
class MotionEstimator
{
public:
    MotionEstimator();

    // Returns fused angle in radians.
    float updateHeading(float gyroZ, float encoderAngle, float dt);

    // Returns fused velocity in m/s.
    float updateVelocity(float accelForward, float encoderVelocity, float dt);

    float getAngle() const;
    float getVelocity() const;
    float getGyroBias() const;
    float getAccelBias() const;

    // Call only while robot stands still: gyroscope reading is then its bias
    void calibrateGyroBias(float gyroZAtRest);

    // Starts new move: keeps heading, velocity from 0, keeps learned sensor biases
    void reset();

private:
    KalmanFilter headingFilter;
    KalmanFilter velocityFilter;
};
