/*
 * Detector.h
 *
 *  Created on: 02.11.2020
 *      Author: AndreR
 */

#pragma once

#include "FrameYUV420.h"
#include "Command.h"

#include <memory>

class Detector
{
public:
    virtual ~Detector() {}

    virtual Command::List processFrame(FrameYUV420* pFrame) =0;

    virtual void processCommand(std::shared_ptr<Command> pCmd) {}
    virtual void setDebugLevel(uint8_t level) {}
};
