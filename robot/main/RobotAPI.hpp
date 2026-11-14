#pragma once

#include "Robot/Robot.hpp"
#include "freertos/queue.h"
#include <chrono>
#include <cstdint>
#include <mutex>


enum RequestType : uint32_t
{
    move,
    rotate,
    cancel // stops current move and drops queued ones, other fields are ignored
};
static_assert(sizeof(RequestType) == 4);

enum ResponseStatus : uint32_t
{
    succeded,
    failed
};
static_assert(sizeof(ResponseStatus) == 4);

struct __attribute__((packed)) Request
{
    uint8_t header;      // 1 byte, always 0x51
    RequestType type;    // 4 bytes
    float distanceInM;   // 4 bytes
    float rotationAngle; // 4 bytes
    uint32_t id;         // 4 bytes
    uint8_t powerMode;   // 1 byte
};
static_assert(sizeof(Request) == 18);

struct __attribute__((packed)) Response
{
    uint32_t id;  // 4 bytes
    ResponseStatus status; // 4 bytes
    uint32_t time; // 4 bytes, ms
};
static_assert(sizeof(Response) == 12);

struct __attribute__((packed)) StateFrame
{
    uint8_t type; // 1 byte, always 0x53
    State state;  // 4 bytes
};
static_assert(sizeof(StateFrame) == 5);

/*
Class used to provide driving interface via computer connection.
It takes Robot and its reference to Data Exchanger.

Requests are received by DataExchanger task and queued (up to maxQueuedRequests).
Own task executes them one at a time in order, so sending sensor data
does not stop during a move. State is Processing until the queue is empty.
Request is answered as failed when the queue is full or robot is in Failure,
when its type is unknown or its power mode is lower than ultraSlow.

Cancel request is not queued, it is handled as soon as it arrives (also in Failure):
all queued requests are dropped and answered as failed, current move stops at
its next control step and is answered as failed, then cancel is answered as succeded.
Robot stays where it stopped, next move starts from there.

Received from computer:
  REQUEST  0x51 – Request (18 bytes including header)
Sent to computer (packets [TYPE][PAYLOAD], little-endian):
  RESPONSE 0x52 – Response (12 bytes), after every request
  STATE    0x53 – State as uint32 (4 bytes), on every change and every 10 s
After a finished move, response is always sent before the state update.
*/
class RobotAPI
{
public:
    RobotAPI(Robot &robot);
    ~RobotAPI();

    // Called by DataExchanger when full Request is received
    void onRequest();

    // Called by DataExchanger in its sending loop, returns 0 when nothing to send
    int sendStatus();

private:
    static constexpr int maxQueuedRequests = 8;
    // Queued requests + current move + cancel, all answered at once on cancel
    static constexpr int maxPendingResponses = maxQueuedRequests + 2;
    static constexpr int responsePacketSize = 1 + sizeof(Response);
    static constexpr int statePacketSize = sizeof(StateFrame);

    Robot &robot;
    DataExchanger &dataExchanger;
    Request request{};
    uint8_t receiveBuffer[sizeof(Request)]{};
    uint8_t sendBuffer[maxPendingResponses * responsePacketSize + statePacketSize]{};

    QueueHandle_t requestQueue;
    QueueHandle_t responseQueue;

    // Taking next request with clearing cancel flag and cancel itself are done
    // under it, so cancel is never lost and never stops a request sent after it
    std::mutex cancelMutex;

    bool stateSent{false};
    State lastSentState{Calibrating};
    std::chrono::steady_clock::time_point lastStateSendTime{};

    static void requestTask(void *arg);
    void processRequests();
    bool executeRequest(const Request &currentRequest);
    void cancelRequests();
    void queueResponse(uint32_t id, ResponseStatus status, uint32_t timeInMs);
};
