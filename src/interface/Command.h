/*
 * Command.h
 *
 *  Created on: 02.11.2020
 *      Author: AndreR
 */

#pragma once

#include <cstdint>
#include <vector>
#include <list>
#include <type_traits>

class Command
{
public:
    typedef std::list<Command> List;

    template<class Data>
    Command(uint16_t cmdId, Data& data);

    Command(uint8_t* pData, size_t length);

    template<class T>
    const T* as();

    uint16_t getId() const { return cmdId_; }
    size_t getLength() const { return data_.size(); }
    const uint8_t* getData() const { return data_.data(); }

private:
    uint16_t cmdId_;
    std::vector<uint8_t> data_;
};

template<class T>
const T* Command::as()
{
    if(sizeof(T) > data_.size())
        return nullptr;

    return reinterpret_cast<T*>(data_.data());
}

template<class Data>
Command::Command(uint16_t cmdId, Data &data)
:cmdId_(cmdId)
{
    static_assert(std::is_standard_layout<Data>::value && std::is_trivial<Data>::value,
                    "Invalid argument. Argument class must have standard-layout and be trivial");

    data_.assign(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(Data));
}
