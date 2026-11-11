#pragma once

#include "Socket.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

enum class PeekResult
{
    Ok,       // `size` bytes copied
    Closed,   // connection closed or broken
    Stalled,  // fewer bytes than `size` and nothing new for PEEK_STALL_MS - not a whole packet
};

// The robot sends every ~50 ms. Must stay well below idle_timeout, otherwise the last bytes of a
// robot that stopped sending are parsed only after the fault and look like a recovery.
constexpr int PEEK_STALL_MS = 300;

inline PeekResult peekBytes(socket_t cfd, uint8_t* buf, int size)
{
    int seen = 0;
    auto lastProgress = std::chrono::steady_clock::now();

    while (true)
    {
        int ret = recv(cfd, (char*)buf, size, MSG_PEEK);

        if (ret < 0)
        {
#ifdef _WIN32
            printf("peek error: %d\n", WSAGetLastError());
#else
            perror("peek");
#endif
            return PeekResult::Closed;
        }
        else if (ret == 0)
        {
            return PeekResult::Closed;
        }

        if (ret >= size)
        {
            return PeekResult::Ok;
        }

        const auto now = std::chrono::steady_clock::now();
        if (ret > seen)
        {
            seen = ret;
            lastProgress = now;
        }
        else if (now - lastProgress > std::chrono::milliseconds(PEEK_STALL_MS))
        {
            return PeekResult::Stalled;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // rest of the packet is on its way
    }
}

// Takes exactly `size` bytes from the socket (after a successful peekBytes they are already there).
inline bool takeBytes(socket_t cfd, uint8_t* buf, int size)
{
    int received = 0;

    while (received < size)
    {
        int ret = recv(cfd, (char*)buf + received, size - received, 0);

        if (ret <= 0)
        {
            return false;
        }

        received += ret;
    }

    return true;
}
