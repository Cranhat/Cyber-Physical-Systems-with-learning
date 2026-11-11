#include "robot_bridge/robot_link.hpp"

#include <cmath>
#include <cstring>
#include <sstream>

namespace robot_bridge
{

double RobotLink::seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

std::string RobotLink::describe(const Command &c)
{
    std::ostringstream out;
    out << "#" << c.id << " " << (c.type == CommandType::Move ? "move(" : "rotate(") << c.value
        << (c.type == CommandType::Move ? " m" : " deg");
    if (c.power != 0)
        out << ", power " << static_cast<int>(c.power);
    out << ")";
    return out.str();
}

void RobotLink::encodeRequest(const Command &c, uint8_t out[kRequestSize])
{
    const uint32_t type = static_cast<uint32_t>(c.type);
    const float distance = c.type == CommandType::Move ? c.value : 0.0F;
    const float angle = c.type == CommandType::Rotate ? c.value : 0.0F;
    out[0] = kRequestType;
    std::memcpy(out + 1, &type, 4);
    std::memcpy(out + 5, &distance, 4);
    std::memcpy(out + 9, &angle, 4);
    std::memcpy(out + 13, &c.id, 4);
    out[17] = c.power;
}

const char *toString(LinkState state)
{
    switch (state)
    {
    case LinkState::WaitingForConnection:
        return "WAITING_FOR_ROBOT";
    case LinkState::Idle:
        return "IDLE";
    case LinkState::Executing:
        return "EXECUTING";
    case LinkState::Unresponsive:
        return "UNRESPONSIVE";
    }
    return "UNKNOWN";
}

RobotLink::RobotLink(LinkConfig config)
    : config_(config), portString_(std::to_string(config.port)), server_(portString_.c_str())
{
    server_.onLidar = [this](const LiData &d) {
        onPacket();
        if (onLidar)
            onLidar(d);
    };
    server_.onMotion = [this](const MotionFrame &m) {
        onPacket();
        if (onMotion)
            onMotion(m);
    };
    server_.onResponse = [this](const ResponseFrame &r) { onResponse(r); };
    server_.onState = [this](RobotState s) { onState(s); };
}

RobotLink::~RobotLink() { stop(); }

void RobotLink::start()
{
    if (running_.exchange(true))
        return;
    receiver_ = std::thread(&RobotLink::receiveLoop, this);
    controller_ = std::thread(&RobotLink::controlLoop, this);
}

void RobotLink::stop()
{
    if (!running_.exchange(false))
        return;
    // wake the receiver if it is blocked in accept()/recv(); repeat in case it was between calls
    while (receiver_.joinable())
    {
        server_.shutdownSock();
        if (receiverDone_)
            receiver_.join();
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    controller_.join();
}

uint32_t RobotLink::enqueue(const Command &command, std::string *error)
{
    const float limit = command.type == CommandType::Move ? kMaxDistanceM : kMaxAngleDeg;
    std::string why;
    if (!std::isfinite(command.value) || command.value == 0.0F || std::fabs(command.value) > limit)
        why = "value must be non-zero and within +/-" + std::to_string(limit);
    const uint8_t power = command.power != 0 ? command.power : config_.power_mode;
    if (why.empty() && power < kMinPower)
        why = "power must be at least " + std::to_string(kMinPower) + " (the robot does not move below it)";

    std::lock_guard<std::mutex> lock(mutex_);
    if (why.empty() && queue_.size() >= config_.max_queue)
        why = "command buffer is full";
    if (!why.empty())
    {
        if (error)
            *error = why;
        return 0;
    }
    Command queued = command;
    queued.id = nextId_;
    queued.power = power;
    nextId_ = nextId_ == UINT32_MAX ? 1 : nextId_ + 1;  // 0 is never used
    queue_.push_back(queued);
    return queued.id;
}

std::size_t RobotLink::clearQueue()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t count = queue_.size();
    for (const Command &c : queue_)
        if (onResult)
            onResult({c, false, "not sent: buffer cleared", std::nullopt});
    queue_.clear();
    return count;
}

LinkStatus RobotLink::status() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    LinkStatus s = status_;
    s.queue_length = queue_.size();
    s.seconds_since_last_packet = havePacket_ ? seconds(Clock::now() - lastPacket_) : -1.0;
    return s;
}

// ---------------------------------------------------------------------------
// Receiving thread
// ---------------------------------------------------------------------------

void RobotLink::receiveLoop()
{
    while (running_)
    {
        if (!server_.connectSock())  // bind + listen + accept (blocks until the robot connects)
        {
            server_.closeSock();
            if (running_)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    say(true, "cannot accept the robot on port " + portString_ + ", retrying");
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            continue;
        }

        onConnected();
        server_.receiveData();  // parses packets until the connection drops
        server_.closeSock();
        onDisconnected();
    }
    receiverDone_ = true;
}

void RobotLink::onConnected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    status_.peer = server_.peer();
    status_.robot_state.reset();
    ++status_.connections;
    havePacket_ = false;
    lastPacket_ = Clock::now();
    setState(LinkState::Idle);
    say(false, "robot connected from " + status_.peer);
}

void RobotLink::onDisconnected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const bool isFault = running_;
    const std::string reason = !dropReason_.empty() ? dropReason_
                               : isFault            ? "robot closed the connection or receive error"
                                                    : "bridge stopped";
    dropReason_.clear();
    status_.peer.clear();
    status_.robot_state.reset();
    havePacket_ = false;
    if (status_.current_command)
        finish(false, "connection lost");
    if (isFault)
    {
        ++status_.faults;
        status_.last_error = reason;
        dropQueue("connection lost");
    }
    say(isFault, "robot disconnected: " + reason);
    setState(LinkState::WaitingForConnection);
}

void RobotLink::onPacket()
{
    std::lock_guard<std::mutex> lock(mutex_);
    havePacket_ = true;
    lastPacket_ = Clock::now();

    if (status_.state == LinkState::Unresponsive)
    {
        say(false, "robot is sending data again - recovered");
        setState(LinkState::Idle);
    }
}

void RobotLink::onResponse(const ResponseFrame &response)
{
    onPacket();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.current_command || status_.current_command->id != response.id)
    {
        // e.g. the answer to a command that already failed here (timeout, reconnect)
        say(true, "response for request #" + std::to_string(response.id) + " that is not executing - ignored");
        return;
    }
    if (response.status == 0)
    {
        finish(true, "", response.time_ms);
        return;
    }
    ++status_.faults;
    status_.last_error = "robot rejected or failed " + describe(*status_.current_command);
    finish(false, "robot reported failure", response.time_ms);
    dropQueue("previous command failed");  // the next commands assume this one was done
}

void RobotLink::onState(RobotState state)
{
    onPacket();
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_.robot_state == state)
        return;
    status_.robot_state = state;
    say(state == RobotState::Failure, std::string("robot state: ") + ::toString(state));

    // The firmware sends the RESPONSE before the state change, so PROCESSING -> READY
    // without a RESPONSE means it was lost - no need to wait for the timeout.
    if (status_.state != LinkState::Executing)
        return;
    if (state == RobotState::Processing)
        sawProcessing_ = true;
    else if (sawProcessing_ && (state == RobotState::Ready || state == RobotState::Idle))
    {
        ++status_.faults;
        status_.last_error = "robot finished " + describe(*status_.current_command) + " without a response";
        finish(false, "robot is READY but sent no response");
        dropQueue("previous command has an unknown result");
    }
}

// ---------------------------------------------------------------------------
// Control thread - sending commands + watchdog
// ---------------------------------------------------------------------------

void RobotLink::controlLoop()
{
    while (running_)
    {
        tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void RobotLink::tick()
{
    bool drop = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = Clock::now();
        const double silence = seconds(now - lastPacket_);
        const double elapsed = seconds(now - sentAt_);

        switch (status_.state)
        {
        case LinkState::WaitingForConnection:
            break;

        case LinkState::Idle:
            if (silence > config_.idle_timeout_s)
            {
                fault("no data from the robot for " + std::to_string(silence) + " s");
            }
            else if (status_.robot_state == RobotState::Failure)
            {
                if (!queue_.empty())
                {
                    status_.last_error = "robot is in FAILURE state";
                    dropQueue(status_.last_error);
                }
            }
            else if (status_.robot_state == RobotState::Processing || status_.robot_state == RobotState::Calibrating)
            {
                // robot still busy (e.g. a request from before a reconnect) - wait for READY
            }
            else if (havePacket_ && !queue_.empty())
            {
                const Command command = queue_.front();
                queue_.pop_front();
                uint8_t request[kRequestSize];
                encodeRequest(command, request);
                status_.current_command = command;
                ++status_.commands_sent;
                if (!server_.sendData(request, sizeof(request)))
                {
                    finish(false, "send failed");
                    status_.last_error = "send failed";
                    dropReason_ = "sending " + describe(command) + " failed";
                    drop = true;
                    break;
                }
                timeout_s_ = (command.type == CommandType::Move ? config_.max_move_time_s
                                                                : config_.max_rotation_time_s) +
                             config_.response_margin_s;
                sentAt_ = now;
                sawProcessing_ = false;
                setState(LinkState::Executing);
                say(false, "sent " + describe(command) + ", " + std::to_string(queue_.size()) + " more in buffer");
            }
            break;

        case LinkState::Executing:
            if (silence > config_.idle_timeout_s)
                fault("robot stopped sending data during " + describe(*status_.current_command));
            else if (elapsed > timeout_s_)
                fault("no response to " + describe(*status_.current_command) + " after " +
                      std::to_string(elapsed) + " s");
            break;

        case LinkState::Unresponsive:
            if (seconds(now - stateSince_) > config_.drop_after_s && dropReason_.empty())
            {
                dropReason_ = "robot unresponsive - closing the connection, waiting for reconnect";
                drop = true;
            }
            break;
        }
    }
    if (drop)
        server_.shutdownSock();  // receiveData() returns, the receiving thread reconnects
}

void RobotLink::fault(const std::string &reason)
{
    ++status_.faults;
    status_.last_error = reason;
    say(true, "FAULT: " + reason + (queue_.empty() ? "" : " (buffer cleared)"));
    if (status_.current_command)
        finish(false, reason);
    dropQueue(reason);
    setState(LinkState::Unresponsive);
}

void RobotLink::finish(bool ok, const std::string &reason, std::optional<uint32_t> robotTimeMs)
{
    if (!status_.current_command)
        return;
    ++(ok ? status_.commands_completed : status_.commands_failed);
    std::string message = describe(*status_.current_command) + (ok ? " completed" : " FAILED: " + reason);
    if (robotTimeMs)
        message += " (robot: " + std::to_string(*robotTimeMs) + " ms)";
    say(!ok, message);
    if (onResult)
        onResult({*status_.current_command, ok, ok ? "" : reason, robotTimeMs});
    status_.current_command.reset();
    if (status_.state == LinkState::Executing)
        setState(LinkState::Idle);
}

// Commands that were never sent are reported as failed (not counted in commands_failed).
void RobotLink::dropQueue(const std::string &reason)
{
    if (queue_.empty())
        return;
    say(true, "dropping " + std::to_string(queue_.size()) + " buffered command(s): " + reason);
    for (const Command &c : queue_)
        if (onResult)
            onResult({c, false, "not sent: " + reason, std::nullopt});
    queue_.clear();
}

void RobotLink::setState(LinkState state)
{
    if (status_.state != state)
    {
        status_.state = state;
        stateSince_ = Clock::now();
    }
}

void RobotLink::say(bool isWarning, const std::string &message)
{
    if (log)
        log(isWarning, message);
}

}  // namespace robot_bridge
