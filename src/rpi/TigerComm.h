#pragma once

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <thread>
#include <deque>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>

#include <termios.h>
#include <unistd.h>
#include <cstring>

#include "interface/Command.h"
#include "interface/CommandTransceiver.h"

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

class TigerComm : public CommandTransceiver
{
public:
    using RemoteTimeCallback = std::function<void(uint32_t)>;

    TigerComm(const std::string& portName, int baudrate, RemoteTimeCallback timeCallback);
    ~TigerComm();

    void write(const Command& pCmd) override;

    std::shared_ptr<Command> read() override;

private:
    bool getBaudrateConstant(int baudrate, int& baudrateConstant) const;
    void receiveThread();
    void writeThread();

    int fileHandle_ = -1;
    termios oldConfig_;

    std::thread receiveThread_;
    std::thread writeThread_;
    bool shutdownRequested_ = false;

    RemoteTimeCallback timeCallback_;

    uint8_t receiveBuffer_[MAXIMUM_MESSAGE_SIZE];
    uint8_t receiveProcessingBuffer_[MAXIMUM_MESSAGE_SIZE];
    uint8_t receiveMessageBuffer_[MAXIMUM_MESSAGE_SIZE];
    uint32_t receiveProcessingBufferPos_ = 0;

    std::deque<std::shared_ptr<Command>> receiveQueue_;
    std::queue<Command> writeQueue_;
    std::mutex receiveQueueMutex_;
    std::mutex writeQueueMutex_;
    std::condition_variable writeMutexCondition_;
};

}
