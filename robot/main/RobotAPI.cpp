#include "RobotAPI.hpp"

#include "freertos/task.h"
#include <cstring>
#include <functional>
#include <stdio.h>

namespace
{
    constexpr uint8_t responsePacketType = 0x52;
    constexpr uint8_t statePacketType = 0x53;
    constexpr auto stateSendInterval = std::chrono::seconds(10);

    // Higher than DataExchanger task, so motor control loop keeps its timing
    constexpr UBaseType_t requestTaskPriority = 6;
    constexpr uint32_t requestTaskStackSize = 4096;
}

RobotAPI::RobotAPI(Robot &robot) : robot(robot), dataExchanger(robot.dataExchanger)
{
    requestQueue = xQueueCreate(maxQueuedRequests, sizeof(Request));
    responseQueue = xQueueCreate(maxPendingResponses, sizeof(Response));

    dataExchanger.appendToReceiving(std::bind(&RobotAPI::onRequest, this), receiveBuffer, sizeof(receiveBuffer));
    dataExchanger.appendToSending(std::bind(&RobotAPI::sendStatus, this), sendBuffer);

    xTaskCreate(requestTask, "robot_requests", requestTaskStackSize, this, requestTaskPriority, nullptr);
}

RobotAPI::~RobotAPI() = default;

void RobotAPI::onRequest()
{
    std::memcpy(&request, receiveBuffer, sizeof(request));

    if (robot.getState() == Failure)
    {
        printf("Robot in failure, request %lu rejected\n", static_cast<unsigned long>(request.id));
        queueResponse(request.id, failed, 0);
        return;
    }

    // Requests wait in queue and are executed one at a time
    if (xQueueSend(requestQueue, &request, 0) != pdTRUE)
    {
        printf("Request queue full, request %lu rejected\n", static_cast<unsigned long>(request.id));
        queueResponse(request.id, failed, 0);
    }
}

void RobotAPI::requestTask(void *arg)
{
    static_cast<RobotAPI *>(arg)->processRequests();
}

void RobotAPI::processRequests()
{
    robot.setState(Ready);

    while (true)
    {
        Request currentRequest{};
        xQueueReceive(requestQueue, &currentRequest, portMAX_DELAY);
        robot.setState(Processing);

        const auto startTime = std::chrono::steady_clock::now();
        const ResponseStatus status = executeRequest(currentRequest) ? succeded : failed;
        const auto elapsed = std::chrono::steady_clock::now() - startTime;
        const auto elapsedInMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

        // Response is queued before state changes, sendStatus relies on this order
        queueResponse(currentRequest.id, status, static_cast<uint32_t>(elapsedInMs));
        if (uxQueueMessagesWaiting(requestQueue) == 0)
        {
            robot.setState(Ready);
        }
    }
}

bool RobotAPI::executeRequest(const Request &currentRequest)
{
    switch (currentRequest.type)
    {
    case move:
        robot.moveForDistance(currentRequest.distanceInM, slow);
        return true;
    case rotate:
        robot.rotateForAngle(currentRequest.rotationAngle, slow);
        return true;
    default:
        printf("Unknown robot request type: %d\n", static_cast<int>(currentRequest.type));
        return false;
    }
}

void RobotAPI::queueResponse(uint32_t id, ResponseStatus status, uint32_t timeInMs)
{
    const Response response{id, status, timeInMs};
    if (xQueueSend(responseQueue, &response, 0) != pdTRUE)
    {
        printf("Response queue full, response %lu dropped\n", static_cast<unsigned long>(id));
    }
}

int RobotAPI::sendStatus()
{
    int length = 0;

    // State is read before responses: response of a finished move is queued
    // before state changes, so it is always sent before the state update
    const State state = robot.getState();

    Response response{};
    while (length + responsePacketSize + statePacketSize <= static_cast<int>(sizeof(sendBuffer)) &&
           xQueueReceive(responseQueue, &response, 0) == pdTRUE)
    {
        sendBuffer[length] = responsePacketType;
        std::memcpy(sendBuffer + length + 1, &response, sizeof(response));
        length += responsePacketSize;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!stateSent || state != lastSentState || now - lastStateSendTime >= stateSendInterval)
    {
        const uint32_t stateValue = state;
        sendBuffer[length] = statePacketType;
        std::memcpy(sendBuffer + length + 1, &stateValue, sizeof(stateValue));
        length += statePacketSize;

        stateSent = true;
        lastSentState = state;
        lastStateSendTime = now;
    }

    return length;
}
