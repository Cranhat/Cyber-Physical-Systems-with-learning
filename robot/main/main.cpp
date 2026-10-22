#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

#include <chrono>
#include <cstdint>
#include <thread>
#include <stdio.h>

#include "Robot/Robot.hpp"
#include "RobotAPI.hpp"
#include "Motor/MotorManager.hpp"
#include "Pwm/Pwm.hpp"
#include "DataExchanger/DataExchanger.hpp"
#include <cmath>

constexpr bool wifiEnabled = false;

// Motors stay off, robot is moved by hand and sensor directions are printed
constexpr bool sensorTest = false;

// Wheels lifted: each motor runs forward, then reverse, and encoder ticks are printed
constexpr bool motorTest = false;

// Robot on the floor: rotates 30 deg clockwise, then 30 deg counter-clockwise, every sample is logged
constexpr bool rotationTest = false;

extern "C"
{
    void app_main(void);
}

void app_main()
{
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("\n---initialize device---\n");

    static I2c i2c{0, 0, GPIO_NUM_8, GPIO_NUM_9};
    static Accelerometer accelerometer{i2c};

    static Lidar lidar{GPIO_NUM_43, GPIO_NUM_44, 230400};
    static DataExchanger dataExchanger("10.52.46.13", 3000, "realme 9 Pro+", "c4i39885");
    if (wifiEnabled)
    {
        dataExchanger.connectToWifi();
    }

    static Pwm pwm{10000};
    static Leds leds{pwm}; // empty for now

    gpio_install_isr_service(0);

    static Encoder encoderLeft(1, 7, GPIO_NUM_1, GPIO_NUM_2, 0.016, 50.0F / 50.0F); // motor 1 {10, 11}, encoders A - 1, B - 2
    static Motor motorLeft(GPIO_NUM_11, GPIO_NUM_10, encoderLeft, pwm);             // Motor 1

    // Right motor is mirrored, so channels are swapped to count positive when driving forward
    static Encoder encoderRight(2, 7, GPIO_NUM_5, GPIO_NUM_4, 0.016, 50.0F / 50.0F); // motor 2 {12, 13}, encoders A - 5, B - 4
    static Motor motorRight(GPIO_NUM_12, GPIO_NUM_13, encoderRight, pwm);            // Motor 2

    static MotorManager motorManager{accelerometer};
    motorManager.addMotor(1, motorLeft);
    motorManager.addMotor(2, motorRight);

    static Robot robot{dataExchanger, accelerometer, leds, lidar, motorManager};
    static RobotAPI robotAPI{robot};

    dataExchanger.appendToSending(std::bind(&Lidar::receiveData, &lidar), lidar.data);
    dataExchanger.appendToSending(std::bind(&Accelerometer::receiveData, &accelerometer), accelerometer.data);
    dataExchanger.appendToSending(std::bind(&Encoder::receiveData, &encoderLeft), encoderLeft.data);
    dataExchanger.appendToSending(std::bind(&Encoder::receiveData, &encoderRight), encoderRight.data);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    if (motorTest)
    {
        printf("\n---motor test---\n");
        Motor *testMotors[] = {&motorLeft, &motorRight};
        const char *names[] = {"left", "right"};
        for (int i = 0; i < 2; i++)
        {
            for (bool forward : {true, false})
            {
                const int64_t start = testMotors[i]->getEncoderTicks();
                testMotors[i]->setDuty(forward, 150);
                vTaskDelay(pdMS_TO_TICKS(1000));
                testMotors[i]->stop();
                vTaskDelay(pdMS_TO_TICKS(500));
                printf("%s %s: %lld ticks\n", names[i], forward ? "forward" : "reverse",
                       static_cast<long long>(testMotors[i]->getEncoderTicks() - start));
            }
        }
        printf("---motor test done---\n");
        return;
    }

    if (sensorTest)
    {
        // Rotate robot clockwise by hand (seen from above): both headings must grow positive.
        // Push robot forward by hand: both wheel distances and velocity must be positive.
        printf("\n---sensor test---\n");
        constexpr float dt = 0.020f;
        constexpr float wheelTrack = 0.162f;
        constexpr float radToDeg = 180.0f / 3.14159265f;
        const int64_t leftStart = encoderLeft.getPulseCount();
        const int64_t rightStart = encoderRight.getPulseCount();
        float gyroHeading = 0.0f;
        float velocity = 0.0f;
        int sample = 0;
        TickType_t lastWakeTime = xTaskGetTickCount();

        while (true)
        {
            float gyroZ = 0.0f;
            float accelForward = 0.0f;
            accelerometer.readMotion(gyroZ, accelForward);
            gyroHeading += -1.0f * gyroZ * dt; // same gyroDirection as MotorManager
            velocity += accelForward * dt;

            if (++sample % 10 == 0)
            {
                const float left = encoderLeft.getDistanceInMeters(encoderLeft.getPulseCount() - leftStart);
                const float right = encoderRight.getDistanceInMeters(encoderRight.getPulseCount() - rightStart);
                const float encoderHeading = (left - right) / wheelTrack;
                printf("gyro heading %7.1f deg | encoder heading %7.1f deg | left %6.3f m | right %6.3f m | accel %6.2f m/s2 | accel velocity %6.2f m/s | right pins A=%d B=%d ticks=%lld\n",
                       gyroHeading * radToDeg, encoderHeading * radToDeg, left, right, accelForward, velocity,
                       gpio_get_level(GPIO_NUM_5), gpio_get_level(GPIO_NUM_4), static_cast<long long>(encoderRight.getPulseCount()));
            }
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
        }
    }

    if (rotationTest)
    {
        printf("\n---rotation test---\n");
        motorManager.logRotation = true;
        printf("--- 30 deg clockwise ---\n");
        motorManager.rotateForAngle(30, normal);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        printf("--- 30 deg counter-clockwise ---\n");
        motorManager.rotateForAngle(-30, normal);
        printf("---rotation test done---\n");
        return;
    }

    if (!wifiEnabled)
    {
        motorManager.moveForDistance(0.5, normal);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        motorManager.rotateForAngle(180, normal);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        motorManager.moveForDistance(0.5, normal);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        motorManager.rotateForAngle(180, normal);
        printf("---driving test done---\n");
        return;
    }

    printf("\n---start tcp client---\n");

    xTaskCreate(
        [](void *arg)
        {
            static_cast<DataExchanger *>(arg)->startTcpClient();
            vTaskDelete(nullptr);
        },
        "tcp_client",
        8192,
        &dataExchanger,
        5,
        nullptr);

    printf("---end --- \n");
};
