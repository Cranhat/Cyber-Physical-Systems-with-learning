#include "VelocityEkf.hpp"
#include <cmath>

namespace
{
    constexpr float pi = 3.14159265f;

    // Process noise Q
    constexpr float q_pos       = 0.001f;  // position uncertainty (m²)
    constexpr float q_yaw       = 0.005f;  // orientation model uncertainty (rad²)
    constexpr float q_velocity  = 0.01f;   // linear velocity model uncertainty (m/s)²
    constexpr float q_accelBias = 0.001f;  // accel bias drift
    constexpr float q_gyroBias  = 0.0005f; // gyro bias drift

    // Measurement noise R
    constexpr float r_velocity = 0.05f; // encoder linear velocity noise
    constexpr float r_yaw      = 0.02f; // encoder differential yaw noise
}

VelocityEkf::VelocityEkf()
{
    reset();

    // Process noise Q: 6x6 diagonal
    Q(0, 0) = q_pos;
    Q(1, 1) = q_pos;
    Q(2, 2) = q_yaw;
    Q(3, 3) = q_velocity;
    Q(4, 4) = q_accelBias;
    Q(5, 5) = q_gyroBias;

    // Measurement noise R: 2x2 diagonal
    R(0, 0) = r_velocity;
    R(0, 1) = 0.0f;
    R(1, 0) = 0.0f;
    R(1, 1) = r_yaw;

    // Measurement matrix H: 2x6
    // z = [v_enc, theta_enc]^T -> observes x[3] (v) and x[2] (theta)
    H(0, 3) = 1.0f;
    H(1, 2) = 1.0f;

    // Identity matrix I: 6x6
    for (int i = 0; i < 6; ++i)
    {
        I(i, i) = 1.0f;
    }
}

float VelocityEkf::update(float imuAccel, float imuGyroZ, float encoderVel, float encoderYaw, float dt)
{
    const float theta = x.data[2];
    const float v = x.data[3];
    const float cosTheta = std::cos(theta);
    const float sinTheta = std::sin(theta);

    // --- 1. Non-linear Prediction Step ---
    // State propagation:
    // x += v * cos(theta) * dt
    // y += v * sin(theta) * dt
    // theta += (gyroZ - b_g) * dt
    // v += (accel - b_a) * dt
    x.data[0] += v * cosTheta * dt;
    x.data[1] += v * sinTheta * dt;
    x.data[2] += (imuGyroZ - x.data[5]) * dt;
    x.data[3] += (imuAccel - x.data[4]) * dt;

    // Jacobian matrix F (6x6):
    dspm::Mat F(6, 6);
    for (int i = 0; i < 36; ++i) F.data[i] = 0.0f;
    F(0, 0) = 1.0f;
    F(0, 2) = -v * sinTheta * dt;
    F(0, 3) = cosTheta * dt;

    F(1, 1) = 1.0f;
    F(1, 2) = v * cosTheta * dt;
    F(1, 3) = sinTheta * dt;

    F(2, 2) = 1.0f;
    F(2, 5) = -dt;

    F(3, 3) = 1.0f;
    F(3, 4) = -dt;

    F(4, 4) = 1.0f;
    F(5, 5) = 1.0f;

    // Predict covariance: P_pred = F * P * F^T + Q
    P = F * P * F.t() + Q;

    // --- 2. Measurement Update Step ---
    // Innovation: y = z - H * x
    const float y0 = encoderVel - x.data[3];
    float y1 = encoderYaw - x.data[2];
    while (y1 > pi) y1 -= 2.0f * pi;
    while (y1 < -pi) y1 += 2.0f * pi;

    // S = H * P * H^T + R  (2x2)
    // Row 0 corresponds to state 3 (v), Row 1 corresponds to state 2 (theta)
    const float S00 = P(3, 3) + R(0, 0);
    const float S01 = P(3, 2);
    const float S10 = P(2, 3);
    const float S11 = P(2, 2) + R(1, 1);

    const float det = S00 * S11 - S01 * S10;
    if (std::abs(det) > 1e-6f)
    {
        const float invDet = 1.0f / det;
        const float invS00 =  S11 * invDet;
        const float invS01 = -S01 * invDet;
        const float invS10 = -S10 * invDet;
        const float invS11 =  S00 * invDet;

        // Kalman gain: K = P * H^T * S^-1  (6x2)
        // Column 0 of P*H^T is P(:, 3), Column 1 is P(:, 2)
        dspm::Mat K(6, 2);
        for (int i = 0; i < 6; ++i)
        {
            const float p_i3 = P(i, 3);
            const float p_i2 = P(i, 2);
            K(i, 0) = p_i3 * invS00 + p_i2 * invS10;
            K(i, 1) = p_i3 * invS01 + p_i2 * invS11;
        }

        // State update: x = x + K * y
        for (int i = 0; i < 6; ++i)
        {
            x.data[i] += K(i, 0) * y0 + K(i, 1) * y1;
        }

        // Covariance update: P = (I - K * H) * P
        dspm::Mat IKH(6, 6);
        for (int i = 0; i < 36; ++i) IKH.data[i] = 0.0f;
        for (int i = 0; i < 6; ++i)
        {
            IKH(i, i) = 1.0f;
            IKH(i, 3) -= K(i, 0);
            IKH(i, 2) -= K(i, 1);
        }

        P = IKH * P;
    }

    return x.data[3]; // fused forward velocity
}

float VelocityEkf::getX() const          { return x.data[0]; }
float VelocityEkf::getY() const          { return x.data[1]; }
float VelocityEkf::getYaw() const        { return x.data[2]; }
float VelocityEkf::getYawDegrees() const { return x.data[2] * 180.0f / pi; }
float VelocityEkf::getVelocity() const   { return x.data[3]; }
float VelocityEkf::getAccelBias() const  { return x.data[4]; }
float VelocityEkf::getGyroBias() const   { return x.data[5]; }

void VelocityEkf::setPose(float newX, float newY, float newYaw)
{
    x.data[0] = newX;
    x.data[1] = newY;
    x.data[2] = newYaw;
}

void VelocityEkf::reset()
{
    for (int i = 0; i < 6; ++i)
    {
        x.data[i] = 0.0f;
    }
    for (int i = 0; i < 36; ++i)
    {
        P.data[i] = 0.0f;
    }
    for (int i = 0; i < 6; ++i)
    {
        P(i, i) = 1.0f;
    }
}
