#pragma once

#include <cstdint>
#include <cstring>

struct Packet {
public:
    const uint8_t* data;
    uint16_t length;

    Packet() = default;
    Packet(const uint8_t* data, uint16_t length) : data(data), length(length){}
};