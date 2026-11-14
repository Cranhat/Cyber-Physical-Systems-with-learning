#include "RobotAPI.hpp"

#include "freertos/task.h"
#include <cstring>
#include <functional>
#include <stdio.h>

namespace
{
    constexpr uint8_t requestPacketType = 0x51;
    constexpr uint8_t responsePacketType = 0x52;
    constexpr uint8_t statePacketType = 0x53;
    constexpr auto stateSendInterval = std::chrono::seconds(10);

    constexpr UBaseType_t requestTaskPriority = 6;
    constexpr uint32_t requestTaskStackSize = 4096;
}

RobotAPI::RobotAPI(Robot &robot) : robot(robot), dataExchanger(robot.dataExchanger)
{
    requestQueue = xQueueCreate(maxQueuedRequests, sizeof(Request));
    responseQueue = xQueueCreate(maxPendingResponses, sizeof(Response));

    dataExchanger.appendToReceiving(std::bind(&RobotAPI::onRequest, this), receiveBuffer, sizeof(receiveBuffer), requestPacketType);
    dataExchanger.appendToSending(std::bind(&RobotAPI::sendStatus, this), sendBuffer);

    xTaskCreate(requestTask, "robot_requests", requestTaskStackSize, this, requestTaskPriority, nullptr);
}

RobotAPI::~RobotAPI() = default;

void RobotAPI::onRequest()
{
    std::memcpy(&request, receiveBuffer, sizeof(request));

    // Handled right away, queued it would wait for the move it should stop
    if (request.type == cancel)
    {
        cancelRequests();
        queueResponse(request.id, succeded, 0);
        return;
    }

    if (robot.getState() == Failure)
    {
        printf("Robot in failure, request %lu rejected\n", static_cast<unsigned long>(request.id));
        queueResponse(request.id, failed, 0);
        return;
    }
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
        // Only waits, request is taken under the mutex together with clearing the cancel flag
        xQueuePeek(requestQueue, &currentRequest, portMAX_DELAY);
        {
            std::lock_guard<std::mutex> lock(cancelMutex);
            if (xQueueReceive(requestQueue, &currentRequest, 0) != pdTRUE)
            {
                // Dropped by cancel in the meantime
                if (uxQueueMessagesWaiting(requestQueue) == 0)
                {
                    robot.setState(Ready);
                }
                continue;
            }
            robot.clearCancel();
        }
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
    // Robot does not move with lower duty, move would only end with timeout
    if (currentRequest.powerMode < ultraSlow)
    {
        printf("Power mode %u too low for request %lu\n", currentRequest.powerMode, static_cast<unsigned long>(currentRequest.id));
        return false;
    }
    const PowerMode powerMode = static_cast<PowerMode>(currentRequest.powerMode);

    // Cancelled move returns false and is answered as failed
    switch (currentRequest.type)
    {
    case move:
        return robot.moveForDistance(currentRequest.distanceInM, powerMode);
    case rotate:
        return robot.rotateForAngle(currentRequest.rotationAngle, powerMode);
    default:
        printf("Unknown robot request type: %d\n", static_cast<int>(currentRequest.type));
        return false;
    }
}

void RobotAPI::cancelRequests()
{
    std::lock_guard<std::mutex> lock(cancelMutex);

    Request dropped{};
    while (xQueueReceive(requestQueue, &dropped, 0) == pdTRUE)
    {
        printf("Request %lu cancelled\n", static_cast<unsigned long>(dropped.id));
        queueResponse(dropped.id, failed, 0);
    }

    // Current move stops at its next control step, processRequests answers it as failed
    robot.cancelMove();
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
        const StateFrame frame{statePacketType, state};
        std::memcpy(sendBuffer + length, &frame, sizeof(frame));
        length += statePacketSize;

        stateSent = true;
        lastSentState = state;
        lastStateSendTime = now;
    }

    return length;
}
