#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>

inline std::mutex& packetDumpMutex()
{
    static std::mutex dump_mutex;
    return dump_mutex;
}

inline void resetPacketDump(const char* path = "dump.txt")
{
    std::lock_guard<std::mutex> lock(packetDumpMutex());
    std::ofstream out(path, std::ios::trunc);
}

inline void writePacketDump(uint8_t type, const uint8_t* payload, size_t payload_size, const char* path = "dump.txt")
{
    std::lock_guard<std::mutex> lock(packetDumpMutex());
    std::ofstream out(path, std::ios::app);

    out << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (unsigned int)type;

    for (size_t i = 0; i < payload_size; i++)
    {
        out << ' ' << std::setw(2) << (unsigned int)payload[i];
    }

    out << '\n';
}
