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

bool Robot::moveForDistance(float distanceInMeters, PowerMode powerMode)
{
    return motorManager.moveForDistance(distanceInMeters, powerMode);
}

bool Robot::rotateForAngle(float angleInDegrees, PowerMode powerMode)
{
    return motorManager.rotateForAngle(angleInDegrees, powerMode);
}

void Robot::stopMotors()
{
    motorManager.stop();
}

void Robot::cancelMove()
{
    motorManager.cancel();
}

void Robot::clearCancel()
{
    motorManager.clearCancel();
}

void Robot::setState(State newState)
{
    state.store(newState);
}

State Robot::getState() const
{
    return state.load();
}

