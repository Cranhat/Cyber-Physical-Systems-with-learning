#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(float estimateProcessNoise, float sensorBiasProcessNoise, float measurementNoise, float initialSensorBiasVariance)
    : estimateProcessNoise(estimateProcessNoise),
      sensorBiasProcessNoise(sensorBiasProcessNoise),
      measurementNoise(measurementNoise),
      sensorBiasVariance(initialSensorBiasVariance)
{
}

float KalmanFilter::update(float imuRate, float encoderMeasurement, float dt)
{
    // --- 1. Prediction Step (IMU) ---
    estimate += (imuRate - sensorBias) * dt;

    // P = F * P * F^T + Q
    estimateVariance += -2.0f * dt * crossCovariance + dt * dt * sensorBiasVariance + estimateProcessNoise;
    crossCovariance -= dt * sensorBiasVariance;
    sensorBiasVariance += sensorBiasProcessNoise;

    // --- 2. Measurement Update Step (encoders) ---
    const float innovation = encoderMeasurement - estimate;
    const float innovationVariance = estimateVariance + measurementNoise;
    const float estimateGain = estimateVariance / innovationVariance;
    const float sensorBiasGain = crossCovariance / innovationVariance;

    estimate += estimateGain * innovation;
    sensorBias += sensorBiasGain * innovation;

    // P = (I - K * H) * P
    sensorBiasVariance -= sensorBiasGain * crossCovariance;
    crossCovariance -= estimateGain * crossCovariance;
    estimateVariance -= estimateGain * estimateVariance;

    return estimate;
}

float KalmanFilter::getEstimate() const
{
    return estimate;
}

float KalmanFilter::getSensorBias() const
{
    return sensorBias;
}

void KalmanFilter::setSensorBias(float newSensorBias)
{
    sensorBias = newSensorBias;
}

void KalmanFilter::reset(float startEstimate)
{
    // Each move starts exactly at startEstimate, sensor bias and its variance are kept
    estimate = startEstimate;
    estimateVariance = 0.0f;
    crossCovariance = 0.0f;
}
