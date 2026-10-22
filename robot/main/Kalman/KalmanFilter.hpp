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
    KalmanFilter(float estimateProcessNoise, float sensorBiasProcessNoise, float measurementNoise, float initialSensorBiasVariance);

    float update(float imuRate, float encoderMeasurement, float dt);

    float getEstimate() const;
    float getSensorBias() const;
    void setSensorBias(float newSensorBias);

    void reset(float startEstimate = 0.0f);

private:
    const float estimateProcessNoise; // uncertainty added to estimate per step (Q)
    const float sensorBiasProcessNoise; // sensor bias drift per step (Q)
    const float measurementNoise; // encoder measurement uncertainty (R)

    float estimate{0.0f};
    float sensorBias{0.0f};
    float estimateVariance{0.0f};
    float crossCovariance{0.0f};
    float sensorBiasVariance{0.0f};
};
