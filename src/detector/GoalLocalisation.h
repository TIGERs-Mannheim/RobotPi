/*
 * WallLocalisation.h
 *
 *  Created on: 22.05.2021
 *      Author: FelixW
 */

#pragma once

#include "interface/Detector.h"
#include "util/Camera.h"
#include "util/TimeSync.h"

class GoalLocalisation : public Detector
{
public:
    GoalLocalisation(std::shared_ptr<TimeSync> pTimeSync);

    Command::List processFrame(FrameYUV420* pFrame) override;
    void processCommand(std::shared_ptr<Command> pCmd) override;
    void setDebugLevel(uint8_t level) override { debugLevel_ = level; }

private:
    Camera camera_;

    uint8_t debugLevel_;

    std::shared_ptr<TimeSync> pTimeSync_;
};
