#pragma once

#include "Robot/Robot.hpp"
#include "freertos/queue.h"
#include <chrono>
#include <cstdint>


enum RequestType : uint32_t
{
    move,
    rotate
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
    static constexpr int maxPendingResponses = maxQueuedRequests;
    static constexpr int responsePacketSize = 1 + sizeof(Response);
    static constexpr int statePacketSize = sizeof(StateFrame);

    Robot &robot;
    DataExchanger &dataExchanger;
    Request request{};
    uint8_t receiveBuffer[sizeof(Request)]{};
    uint8_t sendBuffer[maxPendingResponses * responsePacketSize + statePacketSize]{};

    QueueHandle_t requestQueue;
    QueueHandle_t responseQueue;

    bool stateSent{false};
    State lastSentState{Calibrating};
    std::chrono::steady_clock::time_point lastStateSendTime{};

    static void requestTask(void *arg);
    void processRequests();
    bool executeRequest(const Request &currentRequest);
    void queueResponse(uint32_t id, ResponseStatus status, uint32_t timeInMs);
};
