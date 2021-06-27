#pragma once

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <thread>
#include <deque>
#include <mutex>
#include <functional>

#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "interface/Command.h"

namespace rpi
{

constexpr size_t MAXIMUM_MESSAGE_SIZE = 512;

#define COBSMaxStuffedSize(size) (size+size/208+1)

/**
 * Stuff bytes and remove zeros.
 *
 * @param pIn Data to be stuffed.
 * @param sizeIn Input data size.
 * @param pOut Stuffed output data.
 * @param sizeOut Maximum output size.
 * @param pBytesWritten Actual size of stuffed data, pass 0 if not needed.
 */
int16_t COBSEncode(const uint8_t* pIn, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut, uint32_t* pBytesWritten);

/**
 * Unstuff COBS data.
 *
 * @param pIn Stuffed data.
 * @param sizeIn Stuffed data size.
 * @param pOut Unstuffed data.
 * @param sizeOut Maximum output size.
 * @param pBytesWritten Actual unstuffed data size.
 */
int16_t COBSDecode(const uint8_t* pIn, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut, uint32_t* pBytesWritten);

class TigerComm
{
public:
    using RemoteTimeCallback = std::function<void(uint32_t)>;

    TigerComm();
    ~TigerComm();

    bool open(const std::string& portName, int baudrate);
    void close();

    template<class Data>
    void write(uint16_t commandId, const Data& data);

    void write(const Command& cmd);

    std::shared_ptr<Command> read();

    void setRemoteTimeCallback(RemoteTimeCallback cb) { timeCallback_ = cb; }

private:
    void write(uint16_t commandId, const void* pData, size_t dataLength);

    bool getBaudrateConstant(int baudrate, int& baudrateConstant) const;
    void receiveThread();

    int fileHandle_;
    termios oldConfig_;

    std::thread receiveThread_;
    bool shutdownRequested_;

    RemoteTimeCallback timeCallback_;

    uint8_t receiveBuffer_[MAXIMUM_MESSAGE_SIZE];
    uint8_t receiveProcessingBuffer_[MAXIMUM_MESSAGE_SIZE];
    uint8_t receiveMessageBuffer_[MAXIMUM_MESSAGE_SIZE];
    uint32_t receiveProcessingBufferPos_;

    std::deque<std::shared_ptr<Command>> receiveQueue_;
    std::mutex receiveQueueMutex_;
    std::mutex writeMutex_;
};

template<class Data>
void TigerComm::write(uint16_t commandId, const Data& data)
{
    write(commandId, &data, sizeof(Data));
}

}
