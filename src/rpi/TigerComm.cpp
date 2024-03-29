#include "TigerComm.h"

#include <iostream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <utility>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "util/crc.h"
#include "commands.h"

using namespace std;

namespace rpi
{

TigerComm::TigerComm(const std::string& portName, int baudrate, RemoteTimeCallback timeCallback): timeCallback_(std::move(timeCallback))
{
    if(!getBaudrateConstant(baudrate, baudrate))
        return;

    //open port
    fileHandle_ = open(portName.c_str(), O_RDWR | O_NOCTTY);
    if(fileHandle_ == -1)
    {
        cerr << "Error while opening serial port " << portName << ": " << strerror(errno);
        return;
    }

    bzero(&oldConfig_, sizeof(oldConfig_));
    int ret = tcgetattr(fileHandle_, &oldConfig_); // save current port settings
    if(ret == -1)
    {
        cerr << "Error during tcgetattr: " << strerror(errno);
        return;
    }

    if(!isatty(fileHandle_))
    {
        cerr << "File descriptor " << fileHandle_ << " is NOT a serial port\n";
        return;
    }

    struct termios newtio{};  //structure for port settings
    bzero(&newtio, sizeof(newtio));
    //set data connection to 8N1, no flow control
    newtio.c_cflag = baudrate | CS8 | CREAD | CLOCAL;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 1; // inter-character timer used, in 1/10 seconds
    newtio.c_cc[VMIN] = 0; // blocking read

    tcflush(fileHandle_, TCIFLUSH);
    ret = tcsetattr(fileHandle_, TCSANOW, &newtio);
    if(ret == -1)
    {
        cerr << "Error during tcsetattr: " << strerror(errno);
        return;
    }

    // set serial low latency flag
    serial_struct serial;
    ioctl(fileHandle_, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fileHandle_, TIOCSSERIAL, &serial);

    receiveThread_ = std::thread(&TigerComm::receiveThread, this);
    writeThread_ = std::thread(&TigerComm::writeThread, this);

    cout << "Successfully opened " << portName << " with baudrate define " << baudrate << ". Fd is " << fileHandle_ << endl;
}

TigerComm::~TigerComm()
{
    if(fileHandle_ != -1)
        close(fileHandle_);

    shutdownRequested_ = true;

    if(receiveThread_.joinable())
        receiveThread_.join();

    writeMutexCondition_.notify_one();
    if(writeThread_.joinable())
        writeThread_.join();

    tcsetattr(fileHandle_, TCSANOW, &oldConfig_);
    fileHandle_ = -1;
}

bool TigerComm::getBaudrateConstant(int baudrate, int& baudrateConstant) const
{
    switch(baudrate)
    {
        case 1200:
            baudrateConstant = B1200;
            break;
        case 1800:
            baudrateConstant = B1800;
            break;
        case 9600:
            baudrateConstant = B9600;
            break;
        case 19200:
            baudrateConstant = B19200;
            break;
        case 38400:
            baudrateConstant = B38400;
            break;
        case 57600:
            baudrateConstant = B57600;
            break;
        case 115200:
            baudrateConstant = B115200;
            break;
        case 460800:
            baudrateConstant = B460800;
            break;
        case 921600:
            baudrateConstant = B921600;
            break;
        case 1000000:
            baudrateConstant = B1000000;
            break;
        case 1152000:
            baudrateConstant = B1152000;
            break;
        case 1500000:
            baudrateConstant = B1500000;
            break;
        case 2000000:
            baudrateConstant = B2000000;
            break;
        case 2500000:
            baudrateConstant = B2500000;
            break;
        case 3000000:
            baudrateConstant = B3000000;
            break;
        case 3500000:
            baudrateConstant = B3500000;
            break;
        case 4000000:
            baudrateConstant = B4000000;
            break;
        default:
            cerr << baudrate << " does not match any baudrate constant available.";
            return false;
    }

    return true;
}

void TigerComm::receiveThread()
{
    while(!shutdownRequested_)
    {
        int bytesRead = ::read(fileHandle_, receiveBuffer_, MAXIMUM_MESSAGE_SIZE);

        if(bytesRead < 1)
            continue;

        int receiveBufferPos = 0;

        for(; receiveBufferPos < bytesRead; ++receiveBufferPos)
        {
            const uint8_t data = receiveBuffer_[receiveBufferPos];
            if(data != 0)
            {
                receiveProcessingBuffer_[receiveProcessingBufferPos_] = data;
                ++receiveProcessingBufferPos_;

                if(receiveProcessingBufferPos_ >= MAXIMUM_MESSAGE_SIZE)
                {
                    receiveProcessingBufferPos_ = 0;
                    cerr << "COM RX buffer overflow\n";
                    return;
                }
            }
            else
            {
                if(receiveProcessingBufferPos_ == 0)
                {
                    continue;
                }

                uint32_t decodedSize;
                int16_t result = COBSDecode(receiveProcessingBuffer_, receiveProcessingBufferPos_, receiveMessageBuffer_,
                                MAXIMUM_MESSAGE_SIZE, &decodedSize);
                receiveProcessingBufferPos_ = 0;
                if(result)
                {
                    cerr << "RX COBS decoding error: " << result << endl;
                    continue;
                }

                if(decodedSize >= sizeof(uint16_t)+sizeof(uint32_t))
                {
                    if(CRC32CalcChecksum(receiveMessageBuffer_, decodedSize) == CRC32_MAGIC_NUMBER)
                    {
                        auto cmd = std::make_shared<Command>(receiveMessageBuffer_, decodedSize-sizeof(uint32_t));
                        if(cmd->getId() == CMD_EXT_REMOTE_TIME)
                        {
                            auto rt = cmd->as<ExtRemoteTime>();
                            if(rt && timeCallback_)
                            {
                                (timeCallback_)(rt->timestampUs);
                            }
                        }
                        else
                        {
                            std::lock_guard<std::mutex> lock(receiveQueueMutex_);
                            receiveQueue_.emplace_back(cmd);
                        }
                    }
                    else
                    {
//                        cerr << "CRC error\n";
                    }
                }
                else
                {
//                    cerr << "RX encoded message size < 2, this should not happen\n";
                }
            }
        }

    }
}

void TigerComm::writeThread()
{
    while(!shutdownRequested_)
    {
        Command command;
        {
            std::unique_lock<std::mutex> lock(writeQueueMutex_);
            writeMutexCondition_.wait(lock, [this] { return shutdownRequested_ || !writeQueue_.empty(); });
            if(shutdownRequested_)
                return;

            command = writeQueue_.front();
            writeQueue_.pop();
        }

        // combine commandId, message data, and CRC in one buffer
        uint16_t commandId = command.getId();
        std::vector<uint8_t> msgData(command.getLength() + sizeof(uint16_t) + sizeof(uint32_t));

        memcpy(msgData.data(), &commandId, sizeof(uint16_t));
        memcpy(msgData.data() + sizeof(uint16_t), command.getData(), command.getLength());
        uint32_t crc32 = CRC32CalcChecksum(msgData.data(), sizeof(uint16_t)+command.getLength());
        memcpy(msgData.data() + sizeof(uint16_t) + command.getLength(), &crc32, sizeof(uint32_t));

        // use COBS encoding and add delimiter ('0')
        std::vector<uint8_t> txData(COBSMaxStuffedSize(msgData.size()) + 1);

        uint32_t encodedSize;
        int16_t result = COBSEncode(msgData.data(), msgData.size(), txData.data(), txData.size(), &encodedSize);
        if(result)
        {
            cerr << "COBS encoding error: " << result << " while processing command id: " << commandId << endl;
            return;
        }

        txData[encodedSize] = 0;
        encodedSize++;

        ::write(fileHandle_, txData.data(), encodedSize);
    }
}

std::shared_ptr<Command> TigerComm::read()
{
    std::lock_guard<std::mutex> lock(receiveQueueMutex_);

    if(receiveQueue_.empty())
        return nullptr;

    std::shared_ptr<Command> pMsg = receiveQueue_.front();
    receiveQueue_.pop_front();

    return pMsg;
}

void TigerComm::write(const Command& pCmd)
{
    std::unique_lock<std::mutex> lock(writeQueueMutex_);
    writeQueue_.push(pCmd);
    writeMutexCondition_.notify_one();
}

/*
 * Taken from:  PPP Consistent Overhead Byte Stuffing (COBS)
 * http://tools.ietf.org/html/draft-ietf-pppext-cobs-00.txt
 *
 * - Removed PPP flag extraction (not required and modifies data).
 * - Changed function prototypes slightly
 */

#define ERROR_NOT_ENOUGH_MEMORY         0x0001
#define ERROR_COBS_ZERO_SIZE            0x3000

typedef enum
{
    Unused = 0x00, /* Unused (framing character placeholder) */
    DiffZero = 0x01, /* Range 0x01 - 0xD1:                     */
    DiffZeroMax = 0xD1, /* n-1 explicit characters plus a zero    */
    Diff = 0xD2, /* 209 explicit characters, no added zero */
    RunZero = 0xD3, /* Range 0xD3 - 0xDF:                     */
    RunZeroMax = 0xDF, /* 3-15 zeroes                            */
    Diff2Zero = 0xE0, /* Range 0xE0 - 0xFF:                     */
    Diff2ZeroMax = 0xFF, /* 0-31 explicit characters plus 2 zeroes */
} StuffingCode;

/* These macros examine just the top 3/4 bits of the code byte */
#define isDiff2Zero(X) (X >= Diff2Zero)
#define isRunZero(X)   (X >= RunZero && X <= RunZeroMax)

/* Convert from single-zero code to corresponding double-zero code */
#define ConvertZP (Diff2Zero - DiffZero) // = 0xDF = 223

/* Highest single-zero code with a corresponding double-zero code */
#define MaxConvertible (Diff2ZeroMax - ConvertZP) // = 0x20 = 32

int16_t COBSEncode(const uint8_t* pIn, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut, uint32_t* pBytesWritten)
{
    const uint8_t* pEnd = pIn + sizeIn;
    uint8_t code = DiffZero;
    const uint8_t* pOutOrig = pOut;

    uint8_t* pCode = pOut++;

    if(sizeOut < COBSMaxStuffedSize(sizeIn))
        return ERROR_NOT_ENOUGH_MEMORY;

    while(pIn < pEnd)
    {
        uint8_t c = *pIn++; /* Read the next character */

        if(c == 0) // If it's a zero, do one of these operations
        {
            if(isRunZero(code) && code < RunZeroMax) // If in ZRE mode and below max zero count
            {
                code++; // increment ZRE count
            }
            else if(code == Diff2Zero) // If in two zeros and no character state
            {
                code = RunZero; // switch to ZRE state
            }
            else if(code <= MaxConvertible) // If in diffZero state and below max char count for ZPE:
            {
                code += ConvertZP; // switch to ZPE mode
            }
            else // cannot convert to ZPE (>31 chars) or above max ZRE (>15 '0's)
            {
                *pCode = code;  // save code to this' block code position

                pCode = pOut++; // store code position for new block
                code = DiffZero;  // start new block by single encoded zero
            }
        }
        else // else, non-zero; do one of these operations
        {
            if(isDiff2Zero(code))
            {
                *pCode = code - ConvertZP;
                pCode = pOut++;
                code = DiffZero;
            }
            else if(code == RunZero)
            {
                *pCode = Diff2Zero;
                pCode = pOut++;
                code = DiffZero;
            }
            else if(isRunZero(code))
            {
                *pCode = code - 1;
                pCode = pOut++;
                code = DiffZero;
            }

            *pOut++ = c;

            if(++code == Diff)
            {
                *pCode = code;
                pCode = pOut++;
                code = DiffZero;
            }
        }
    }

    // finalize packet, just in case this was the last data to encode
    *pCode = code;
    pCode = pOut++;
    code = DiffZero;

    if(pBytesWritten)
        *pBytesWritten = pOut - pOutOrig - 1;

    return 0;
}

int16_t COBSDecode(const uint8_t* pIn, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut, uint32_t* pBytesWritten)
{
    const uint8_t* pOutOrig = pOut;
    const uint8_t* pEnd = pIn + sizeIn;
    const uint8_t* pLimit = pOut + sizeOut;

    if(sizeIn == 0)
        return ERROR_COBS_ZERO_SIZE;

    while(pIn < pEnd)
    {
        int32_t z, c = *pIn++; // c = code, z = zeros

        if(c == Diff)
        {
            z = 0;
            c--;
        }
        else if(isRunZero(c))
        {
            z = c & 0xF;
            c = 0;
        }
        else if(isDiff2Zero(c))
        {
            z = 2;
            c &= 0x1F;
        }
        else
        {
            z = 1;
            c--;
        }

        while(--c >= 0)
        {
            if(pOut < pLimit)
                *pOut = *pIn;
            ++pOut;
            ++pIn;
        }

        while(--z >= 0)
        {
            if(pOut < pLimit)
                *pOut = 0;
            ++pOut;
        }
    }

    if(pBytesWritten)
        *pBytesWritten = pOut - pOutOrig - 1;

    if(pOut >= pLimit)
        return ERROR_NOT_ENOUGH_MEMORY;

    return 0;
}

} // namespace tigers
