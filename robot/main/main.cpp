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

    static Pwm pwm{10000};
    static Leds leds{pwm}; // empty for now

    gpio_install_isr_service(0);

    static Encoder encoderLeft(1, 7, GPIO_NUM_1, GPIO_NUM_2, 0.016, 50.0F / 53.0F); // motor 1 {10, 11}, encoders A - 1, B - 2
    static Motor motorLeft(GPIO_NUM_11, GPIO_NUM_10, encoderLeft, pwm);             // Motor 1

    static Encoder encoderRight(2, 7, GPIO_NUM_4, GPIO_NUM_5, 0.016, 50.0F / 53.0F); // motor 2 {12, 13}, encoders A - 4, B - 5
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

    // while (true)
    // {
    // }

    printf("---end --- \n");
};
