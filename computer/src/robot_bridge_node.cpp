#include "robot_bridge/robot_bridge_node.hpp"

#include <limits>
#include <stdexcept>

namespace robot_bridge
{

RobotBridgeNode::RobotBridgeNode() : Node("robot_bridge")
{
    LinkConfig config;
    config.port = static_cast<uint16_t>(declare_parameter<int>("port", config.port));
    config.idle_timeout_s = declare_parameter<double>("idle_timeout", config.idle_timeout_s);
    config.drop_after_s = declare_parameter<double>("drop_after", config.drop_after_s);
    config.max_move_time_s = declare_parameter<double>("max_move_time", config.max_move_time_s);
    config.max_rotation_time_s = declare_parameter<double>("max_rotation_time", config.max_rotation_time_s);
    config.response_margin_s = declare_parameter<double>("response_margin", config.response_margin_s);
    config.max_queue = static_cast<std::size_t>(declare_parameter<int>("max_queue", 32));
    const int power = declare_parameter<int>("power_mode", config.power_mode);
    if (power < kMinPower || power > 255)
        throw std::invalid_argument("power_mode must be within " + std::to_string(kMinPower) + "..255");
    config.power_mode = static_cast<uint8_t>(power);
    odomFrame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    baseFrame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    lidarFrame_ = declare_parameter<std::string>("lidar_frame_id", "laser");

    statusPub_ = create_publisher<DiagnosticStatus>("~/status", rclcpp::QoS(1).transient_local());
    resultPub_ = create_publisher<DiagnosticStatus>("~/result", rclcpp::QoS(50).reliable());
    odomPub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    scanPub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

    moveSub_ = create_subscription<Float32>("~/move", 10, [this](const Float32 &m) {
        enqueue({CommandType::Move, m.data});
    });
    rotateSub_ = create_subscription<Float32>("~/rotate", 10, [this](const Float32 &m) {
        enqueue({CommandType::Rotate, m.data});
    });
    clearSrv_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_queue", [this](std_srvs::srv::Trigger::Request::SharedPtr,
                                std_srvs::srv::Trigger::Response::SharedPtr response) {
            response->success = true;
            response->message = "removed " + std::to_string(link_->clearQueue()) + " command(s)";
        });
    statusTimer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { publishStatus(); });

    link_ = std::make_unique<RobotLink>(config);
    link_->log = [this](bool warning, const std::string &m) {
        if (warning)
            RCLCPP_WARN(get_logger(), "%s", m.c_str());
        else
            RCLCPP_INFO(get_logger(), "%s", m.c_str());
    };
    link_->onResult = [this](const CommandResult &r) { publishResult(r); };
    link_->onMotion = [this](const MotionFrame &m) { publishOdometry(m); };
    link_->onLidar = [this](const LiData &f) { addLidarFrame(f); };
    link_->start();
}

RobotBridgeNode::~RobotBridgeNode() { link_->stop(); }  // stop the IO threads before publishers go away

void RobotBridgeNode::enqueue(const Command &command)
{
    std::string error;
    const uint32_t id = link_->enqueue(command, &error);
    if (id == 0)
        RCLCPP_ERROR(get_logger(), "command rejected: %s", error.c_str());
    else
        RCLCPP_INFO(get_logger(), "queued %s", RobotLink::describe({command.type, command.value, id}).c_str());
}

// One message per command: name = request id, level OK / ERROR, message = reason.
void RobotBridgeNode::publishResult(const CommandResult &r)
{
    DiagnosticStatus m;
    m.name = std::to_string(r.command.id);
    m.level = r.ok ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
    m.message = r.ok ? "succeeded" : r.reason;
    auto add = [&m](const std::string &key, const std::string &value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        m.values.push_back(kv);
    };
    add("id", std::to_string(r.command.id));
    add("type", r.command.type == CommandType::Move ? "move" : "rotate");
    add("value", std::to_string(r.command.value));
    add("power", std::to_string(r.command.power));
    add("robot_time_ms", r.robot_time_ms ? std::to_string(*r.robot_time_ms) : "");
    resultPub_->publish(m);
}

void RobotBridgeNode::publishStatus()
{
    const LinkStatus s = link_->status();
    DiagnosticStatus m;
    m.name = "robot_bridge";
    m.hardware_id = s.peer;
    m.level = s.state == LinkState::Unresponsive           ? DiagnosticStatus::ERROR
              : s.state == LinkState::WaitingForConnection ? DiagnosticStatus::WARN
                                                           : DiagnosticStatus::OK;
    m.message = toString(s.state);
    auto add = [&m](const std::string &key, const std::string &value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        m.values.push_back(kv);
    };
    add("robot_state", s.robot_state ? toString(*s.robot_state) : "");
    add("seconds_since_last_packet", std::to_string(s.seconds_since_last_packet));
    add("queue_length", std::to_string(s.queue_length));
    add("current_command", s.current_command ? RobotLink::describe(*s.current_command) : "");
    add("commands_sent", std::to_string(s.commands_sent));
    add("commands_completed", std::to_string(s.commands_completed));
    add("commands_failed", std::to_string(s.commands_failed));
    add("connections", std::to_string(s.connections));
    add("faults", std::to_string(s.faults));
    add("last_error", s.last_error);
    statusPub_->publish(m);
}

void RobotBridgeNode::publishOdometry(const MotionFrame &m)
{
    const double yaw = -static_cast<double>(m.heading);
    const uint64_t timeUs = m.velocity_time_us;

    if (!haveMotion_ || timeUs < lastMotionTimeUs_)
    {
        if (haveMotion_)
            RCLCPP_WARN(get_logger(), "robot time went back (reboot?) - odometry continues from the last pose");
        haveMotion_ = true;
        lastMotionTimeUs_ = timeUs;
        lastYaw_ = yaw;
    }

    const double dt = static_cast<double>(timeUs - lastMotionTimeUs_) * 1e-6;
    double yawRate = 0.0;
    if (dt > 0.0)
    {
        x_ += m.velocity * std::cos(yaw) * dt;
        y_ += m.velocity * std::sin(yaw) * dt;
        yawRate = (yaw - lastYaw_) / dt;
    }
    lastMotionTimeUs_ = timeUs;
    lastYaw_ = yaw;

    nav_msgs::msg::Odometry o;
    o.header.stamp = now();
    o.header.frame_id = odomFrame_;
    o.child_frame_id = baseFrame_;
    o.pose.pose.position.x = x_;
    o.pose.pose.position.y = y_;
    o.pose.pose.orientation.z = std::sin(yaw / 2.0);
    o.pose.pose.orientation.w = std::cos(yaw / 2.0);
    o.twist.twist.linear.x = m.velocity;
    o.twist.twist.angular.z = yawRate;
    odomPub_->publish(o);
}

// LD06 measures clockwise, LaserScan is counter-clockwise.
void RobotBridgeNode::addLidarFrame(const LiData &f)
{
    if (f.startAngle < lastStart_ && points_ > 50)
    {
        scan_.header.stamp = now();
        scan_.header.frame_id = lidarFrame_;
        scan_.angle_min = 0.0F;
        scan_.angle_increment = static_cast<float>(M_PI / 180.0);
        scan_.angle_max = scan_.angle_increment * 359.0F;
        scan_.scan_time = f.speed > 0 ? 360.0F / f.speed : 0.1F;
        scan_.time_increment = scan_.scan_time / 360.0F;
        scan_.range_min = 0.02F;
        scan_.range_max = 12.0F;
        scanPub_->publish(scan_);
    }
    if (f.startAngle < lastStart_ || scan_.ranges.empty())
    {
        scan_.ranges.assign(360, std::numeric_limits<float>::quiet_NaN());
        scan_.intensities.assign(360, 0.0F);
        points_ = 0;
    }
    lastStart_ = f.startAngle;

    const int span = (f.endAngle - f.startAngle + 36000) % 36000;  // 0.01 deg
    for (int i = 0; i < POINT_PER_PACK; ++i)
    {
        const float range = f.point[i].distanceValue / 1000.0F;
        if (range < 0.02F || range > 12.0F)
            continue;
        const double angle = (f.startAngle + span * i / 11.0) / 100.0;
        const int bin = static_cast<int>(std::lround(360.0 - angle)) % 360;
        if (std::isnan(scan_.ranges[bin]) || range < scan_.ranges[bin])
        {
            scan_.ranges[bin] = range;
            scan_.intensities[bin] = f.point[i].confidence;
        }
        ++points_;
    }
}

}  // namespace robot_bridge

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_bridge::RobotBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
