#pragma once

#include "mat.h"

/*
6-State Extended Kalman Filter for 2D Planar Robot Motion fusing:
1. IMU: Linear acceleration (ax) + Angular velocity (gyroZ)
2. Encoders: Wheel linear velocity (v_enc) + Differential heading (theta_enc)

State vector: x = [x, y, theta, v, b_a, b_g]^T
  x[0]: x     – 2D position X (meters)
  x[1]: y     – 2D position Y (meters)
  x[2]: theta – heading / yaw angle (radians)
  x[3]: v     – forward linear velocity (m/s)
  x[4]: b_a   – accelerometer bias (m/s²)
  x[5]: b_g   – gyroscope bias (rad/s)

Non-linear Prediction step:
  x(k+1)     = x(k) + v(k) * cos(theta(k)) * dt
  y(k+1)     = y(k) + v(k) * sin(theta(k)) * dt
  theta(k+1) = theta(k) + (gyroZ - b_g(k)) * dt
  v(k+1)     = v(k) + (accel - b_a(k)) * dt
  b_a(k+1)   = b_a(k)
  b_g(k+1)   = b_g(k)

Update step (driven by encoders):
  z = [v_enc, theta_enc]^T
*/
class VelocityEkf
{
public:
    VelocityEkf();

    // Full 6-state EKF update:
    // imuAccel   – forward acceleration from IMU (m/s²)
    // imuGyroZ   – angular velocity around Z axis from IMU (rad/s)
    // encoderVel – linear velocity from encoders (m/s)
    // encoderYaw – heading angle from differential encoders (rad)
    // dt         – time step in seconds
    // Returns fused linear velocity in m/s.
    float update(float imuAccel, float imuGyroZ, float encoderVel, float encoderYaw, float dt);

    float getX() const;          // 2D position X in meters
    float getY() const;          // 2D position Y in meters
    float getYaw() const;        // heading in radians
    float getYawDegrees() const; // heading in degrees
    float getVelocity() const;   // linear velocity in m/s
    float getAccelBias() const;
    float getGyroBias() const;

    void setPose(float newX, float newY, float newYaw = 0.0f);
    void reset();

private:
    dspm::Mat x{6, 1}; // State [6x1]: [x, y, theta, v, b_a, b_g]
    dspm::Mat P{6, 6}; // Covariance [6x6]
    dspm::Mat Q{6, 6}; // Process noise [6x6]
    dspm::Mat R{2, 2}; // Measurement noise [2x2]
    dspm::Mat H{2, 6}; // Measurement matrix [2x6]
    dspm::Mat I{6, 6}; // Identity [6x6]
};


