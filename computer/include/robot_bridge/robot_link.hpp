#pragma once

#include "TCPserver.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace robot_bridge
{

enum class CommandType : uint32_t
{
    Move = 0,    // value in meters
    Rotate = 1,  // value in degrees
};

struct Command
{
    CommandType type = CommandType::Move;
    float value = 0.0F;
    uint32_t id = 0;     // request id, given by RobotLink::enqueue()
    uint8_t power = 0;   // PWM duty x/255 (robot PowerMode), 0 = LinkConfig::power_mode
};

// PowerMode values from robot/main/Motor/Motor.hpp; the robot rejects requests below kMinPower.
constexpr uint8_t kMinPower = 100;  // ultraSlow (slow = 150, normal = 180, fast = 220, overdrive = 255)

// Outcome of one command: the robot's RESPONSE, or a failure detected on the PC side.
struct CommandResult
{
    Command command;
    bool ok = false;
    std::string reason;           // empty when ok
    std::optional<uint32_t> robot_time_ms;  // from the RESPONSE, if there was one
};

enum class LinkState
{
    WaitingForConnection,
    Idle,
    Executing,
    Unresponsive,
};

const char *toString(LinkState state);

struct LinkConfig
{
    uint16_t port = 3000;
    double idle_timeout_s = 1.0;      // no data for this long -> UNRESPONSIVE
    double drop_after_s = 5.0;        // UNRESPONSIVE this long -> close the connection
    // The robot ends a move / rotation itself after maxMoveTime / maxRotationTime
    // (robot/main/Config.hpp) and answers, so no RESPONSE after this + margin = robot fault.
    double max_move_time_s = 20.0;
    double max_rotation_time_s = 10.0;
    double response_margin_s = 3.0;   // added to the response timeout
    std::size_t max_queue = 32;
    uint8_t power_mode = 150;          // default duty of a command (robot `slow`)
};

struct LinkStatus
{
    LinkState state = LinkState::WaitingForConnection;
    std::optional<RobotState> robot_state;  // last STATE packet of this connection
    std::string peer;
    double seconds_since_last_packet = -1.0;
    std::size_t queue_length = 0;
    std::optional<Command> current_command;
    uint64_t commands_sent = 0, commands_completed = 0, commands_failed = 0;
    uint64_t connections = 0, faults = 0;
    std::string last_error;
};

// Sends commands to the robot one at a time: the next `Request` goes out only after the
// RESPONSE with the id of the previous one (or after it failed on the PC side).
class RobotLink
{
public:
    explicit RobotLink(LinkConfig config);
    ~RobotLink();

    // Set before start(). Sensor callbacks run on the receiving thread. `log` and `onResult`
    // may be called while the internal lock is held, so they must not call back into RobotLink.
    std::function<void(const LiData &)> onLidar;
    std::function<void(const MotionFrame &)> onMotion;  // robot velocity + heading estimate, every loop
    std::function<void(const CommandResult &)> onResult;
    std::function<void(bool isWarning, const std::string &)> log;

    void start();
    void stop();

    // Thread-safe. enqueue() returns the request id, 0 when the command was rejected.
    uint32_t enqueue(const Command &command, std::string *error = nullptr);
    std::size_t clearQueue();
    LinkStatus status() const;

    // 18-byte packed `Request` from robot/main/RobotAPI.hpp:
    // uint8 0x51, uint32 type, float distanceInM, float rotationAngle, uint32 id, uint8 powerMode
    static constexpr uint8_t kRequestType = 0x51;
    static constexpr std::size_t kRequestSize = 18;
    static void encodeRequest(const Command &c, uint8_t out[kRequestSize]);
    static std::string describe(const Command &c);

private:
    using Clock = std::chrono::steady_clock;

    static constexpr float kMaxDistanceM = 2.0F;
    static constexpr float kMaxAngleDeg = 360.0F;

    static double seconds(Clock::duration d);

    void receiveLoop();
    void controlLoop();
    void onConnected();
    void onDisconnected();
    void onPacket();
    void onResponse(const ResponseFrame &response);
    void onState(RobotState state);
    void tick();
    void fault(const std::string &reason);
    void finish(bool ok, const std::string &reason, std::optional<uint32_t> robotTimeMs = std::nullopt);
    void dropQueue(const std::string &reason);
    void setState(LinkState state);
    void say(bool isWarning, const std::string &message);

    LinkConfig config_;
    std::string portString_;  // TCPserver keeps a pointer to it
    TCPserver server_;
    std::thread receiver_, controller_;
    std::atomic<bool> running_{false};
    std::atomic<bool> receiverDone_{false};

    mutable std::mutex mutex_;  // guards everything below
    LinkStatus status_;
    std::deque<Command> queue_;
    uint32_t nextId_ = 1;
    std::string dropReason_;
    bool havePacket_ = false;
    bool sawProcessing_ = false;  // robot reported PROCESSING since the current command was sent
    double timeout_s_ = 0.0;
    Clock::time_point lastPacket_, sentAt_, stateSince_;
};

}  // namespace robot_bridge
