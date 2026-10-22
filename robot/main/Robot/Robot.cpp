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

void Robot::moveForDistance(float distanceInMeters, PowerMode powerMode)
{
    motorManager.moveForDistance(distanceInMeters, powerMode);
}

void Robot::rotateForAngle(float angleInDegrees, PowerMode powerMode)
{
    motorManager.rotateForAngle(angleInDegrees, powerMode);
}

void Robot::stopMotors()
{
    motorManager.stop();
}

void Robot::setState(State newState)
{
    state.store(newState);
}

State Robot::getState() const
{
    return state.load();
}

