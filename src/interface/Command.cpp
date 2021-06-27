/*
 * Command.cpp
 *
 *  Created on: 02.11.2020
 *      Author: AndreR
 */

#include "Command.h"

Command::Command(uint8_t* pData, size_t length)
{
    cmdId_ = *reinterpret_cast<uint16_t*>(pData);
    data_.assign(pData + 2, pData + length);
}
