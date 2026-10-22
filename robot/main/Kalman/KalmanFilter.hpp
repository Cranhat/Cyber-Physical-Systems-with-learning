#pragma once

/*
2-State linear Kalman Filter fusing an IMU rate sensor with encoders.
Same structure is used twice by MotorManager:
  heading:  imuRate = gyroZ (rad/s),  estimate = angle (rad),  encoderMeasurement = encoder angle
  velocity: imuRate = accel (m/s²),   estimate = v (m/s),      encoderMeasurement = encoder velocity

State vector: x = [estimate, sensorBias]^T
  estimate   – estimated quantity, relative to the start of the move
  sensorBias – IMU sensor bias, learned online and kept between moves

Prediction step (IMU):
  estimate(k+1)   = estimate(k) + (imuRate - sensorBias(k)) * dt
  sensorBias(k+1) = sensorBias(k)
  F = [1, -dt]
      [0,   1]
  P = F * P * F^T + Q

Update step (encoders):
  z = estimate,  H = [1, 0]
  K = P * H^T / (H * P * H^T + R)
  x = x + K * (z - estimate)
  P = (I - K * H) * P

Covariance P is symmetric:
  P = [estimateVariance,  crossCovariance]
      [crossCovariance,   sensorBiasVariance]
*/
class KalmanFilter
{
public:
    // estimateProcessNoise   – uncertainty added to estimate per step (Q)
    // sensorBiasProcessNoise – sensor bias drift per step (Q)
    // measurementNoise       – encoder measurement uncertainty (R)
    // initialSensorBiasVariance – how well the bias is known at boot
    KalmanFilter(float estimateProcessNoise, float sensorBiasProcessNoise, float measurementNoise, float initialSensorBiasVariance);

    // imuRate            – IMU rate (gyroZ or forward acceleration)
    // encoderMeasurement – same quantity as estimate, measured by encoders
    // dt                 – time step in seconds
    // Returns fused estimate.
    float update(float imuRate, float encoderMeasurement, float dt);

    float getEstimate() const;
    float getSensorBias() const;
    void setSensorBias(float newSensorBias);

    // Starts new move from startEstimate, keeps learned sensor bias
    void reset(float startEstimate = 0.0f);

private:
    const float estimateProcessNoise;
    const float sensorBiasProcessNoise;
    const float measurementNoise;

    float estimate{0.0f};
    float sensorBias{0.0f};

    float estimateVariance{0.0f};
    float crossCovariance{0.0f};
    float sensorBiasVariance{0.0f};
};
