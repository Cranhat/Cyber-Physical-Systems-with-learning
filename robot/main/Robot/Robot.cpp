#include "Robot/Robot.hpp"

Robot::Robot(DataExchanger &dataExchanger, Accelerometer &accelerometer, Leds &leds, Lidar &lidar, MotorManager &motorManager) : dataExchanger(dataExchanger),
                                                                                                                                 accelerometer(accelerometer),
                                                                                                                                 leds(leds),
                                                                                                                                 lidar(lidar),
                                                                                                                                 motorManager(motorManager)

{
    start();
}

Robot::~Robot()
{
    stop();
}

void Robot::start() {

};

void Robot::stop() {

};

void Robot::moveMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuAccel, float imuGyroZ)
{
    motorManager.moveMotorsForEncoderTicksCount(ticks, powerMode, imuAccel, imuGyroZ);
}

void Robot::rotateMotorsForEncoderTicksCount(int64_t ticks, PowerMode powerMode, float imuGyroZ)
{
    motorManager.rotateMotorsForEncoderTicksCount(ticks, powerMode, imuGyroZ);
}

void Robot::moveWithVelocity(float velocityInMpS, float imuAccel, float imuGyroZ)
{
    motorManager.moveWithVelocity(velocityInMpS, imuAccel, imuGyroZ);
}

void Robot::moveForDistance(float distanceInMeters, float velocityInMpS, float imuAccel, float imuGyroZ)
{
    motorManager.moveForDistance(distanceInMeters, velocityInMpS, imuAccel, imuGyroZ);
}

void Robot::rotateForAngle(float angleInDegrees, float speedMps, float imuGyroZ)
{
    motorManager.rotateForAngle(angleInDegrees, speedMps, imuGyroZ);
}

void Robot::stopMotors()
{
    motorManager.stop();
}

