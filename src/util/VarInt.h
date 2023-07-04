#pragma once

#include <cstdint>
#include <vector>

/*
 Read/Write functionality for variable value integers in LEB128 format
 LEB128 definition: https://en.wikipedia.org/wiki/LEB128
 */

template<typename Container>
inline void encodeVarUInt32(Container& data, uint32_t value)
{
    while(value > 127)
    {
        data.emplace_back((value & 0x7F) + 0x80);
        value >>= 7;
    }
    data.emplace_back(value);
}

template<typename Iterator>
inline bool decodeVarUInt32(Iterator& itr, const Iterator& end, uint32_t& decodedValue)
{
    decodedValue = 0;
    uint8_t position = 0;
    uint8_t byte;
    do
    {
        if(itr == end)
            return false;

        byte = *(itr++);
        decodedValue |= (uint32_t)(byte & 0x7F) << position;
        position+=7;
    } while(byte > 127);

    return true;
}