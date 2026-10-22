idf_component_register(SRCS 
    "Accelerometer/Accelerometer.cpp"
    "DataExchanger/DataExchanger.cpp" 
    "EKF/VelocityEkf.cpp"
    "I2c/I2c.cpp"
    "I2c/I2cSlave.cpp"
    "Led/Led.cpp"
    "Led/Leds.cpp"
    "Lidar/Lidar.cpp"
    "Motor/Controller.cpp"
    "Motor/Encoder.cpp"
    "Motor/Motor.cpp"
    "Motor/MotorManager.cpp"
    "Pwm/Pwm.cpp"
    "Pwm/PwmChannel.cpp"
    "Robot/Robot.cpp"
    "Uart/Uart.cpp"
    "RobotAPI.cpp"
    "main.cpp"
    REQUIRES esp_driver_uart
    REQUIRES esp_driver_i2c
    REQUIRES esp_driver_gpio
    REQUIRES esp_driver_ledc
    REQUIRES esp_wifi
    REQUIRES esp_http_server
    REQUIRES esp_http_client
    REQUIRES esp_eth
    REQUIRES esp_event
    REQUIRES nvs_flash
    REQUIRES esp_timer
    REQUIRES espressif__pid_ctrl
    REQUIRES espressif__esp-dsp
    INCLUDE_DIRS ".")

