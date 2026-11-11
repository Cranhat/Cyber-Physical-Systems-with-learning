#pragma once

#include "robot_bridge/robot_link.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace robot_bridge
{

class RobotBridgeNode : public rclcpp::Node
{
public:
    RobotBridgeNode();
    ~RobotBridgeNode() override;

private:
    using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
    using Float32 = std_msgs::msg::Float32;

    void enqueue(const Command &command);
    void publishStatus();
    void publishResult(const CommandResult &r);
    void publishOdometry(const MotionFrame &m);  // integrates the robot estimates into a pose
    void addLidarFrame(const LiData &f);  // collects LD06 frames into one revolution

    std::string odomFrame_, baseFrame_, lidarFrame_;
    bool haveMotion_ = false;
    uint64_t lastMotionTimeUs_ = 0;
    double lastYaw_ = 0.0;
    double x_ = 0.0, y_ = 0.0;
    sensor_msgs::msg::LaserScan scan_;
    int lastStart_ = -1;
    int points_ = 0;

    rclcpp::Publisher<DiagnosticStatus>::SharedPtr statusPub_, resultPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
    rclcpp::Subscription<Float32>::SharedPtr moveSub_, rotateSub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clearSrv_;
    rclcpp::TimerBase::SharedPtr statusTimer_;
    std::unique_ptr<RobotLink> link_;
};

}  // namespace robot_bridge
