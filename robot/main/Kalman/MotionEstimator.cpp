#include "MotionEstimator.hpp"

namespace
{
    // Heading Kalman filter noise
    constexpr float q_angle    = 0.000001f; // gyroscope integration per step (rad²)
    constexpr float q_gyroBias = 0.0f;      // bias not learned while moving
    constexpr float r_angle    = 0.25f;     // encoder heading, wheel slip and diameter mismatch (rad²)
    constexpr float p_gyroBias = 0.0f;      // bias set by calibrateGyroBias, not by the filter

    // Running average weight of one at-rest gyroscope sample
    constexpr float gyroBiasAveraging = 0.1f;

    // Velocity Kalman filter noise
    constexpr float q_velocity  = 0.0001f;   // accelerometer integration and vibration per step (m/s)²
    constexpr float q_accelBias = 0.000001f; // accelerometer bias drift per step
    constexpr float r_velocity  = 0.0004f;   // encoder velocity (m/s)²
    constexpr float p_accelBias = 0.01f;     // initial accelerometer bias variance, tilt (m/s²)²
}

MotionEstimator::MotionEstimator()
    : headingFilter(q_angle, q_gyroBias, r_angle, p_gyroBias),
      velocityFilter(q_velocity, q_accelBias, r_velocity, p_accelBias)
{
}

float MotionEstimator::updateHeading(float gyroZ, float encoderAngle, float dt)
{
    return headingFilter.update(gyroZ, encoderAngle, dt);
}

float MotionEstimator::updateVelocity(float accelForward, float encoderVelocity, float dt)
{
    return velocityFilter.update(accelForward, encoderVelocity, dt);
}

float MotionEstimator::getAngle() const
{
    return headingFilter.getEstimate();
}

float MotionEstimator::getVelocity() const
{
    return velocityFilter.getEstimate();
}

float MotionEstimator::getGyroBias() const
{
    return headingFilter.getSensorBias();
}

float MotionEstimator::getAccelBias() const
{
    return velocityFilter.getSensorBias();
}

void MotionEstimator::calibrateGyroBias(float gyroZAtRest)
{
    const float gyroBias = headingFilter.getSensorBias();
    headingFilter.setSensorBias(gyroBias + gyroBiasAveraging * (gyroZAtRest - gyroBias));
}

void MotionEstimator::reset()
{
    headingFilter.reset(headingFilter.getEstimate());
    velocityFilter.reset();
}
