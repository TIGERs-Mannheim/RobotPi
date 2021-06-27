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

class WallLocalisation : public Detector
{
public:
    WallLocalisation(std::shared_ptr<TimeSync> pTimeSync);
    
    Command::List processFrame(FrameYUV420* pFrame) override;
    void processCommand(std::shared_ptr<Command> pCmd) override;
    void setDebugLevel(uint8_t level) override { debugLevel_ = level; }
    
    void setFieldRotation(float fieldRotation);

private:
    Camera camera_;

    uint8_t debugLevel_;
    Eigen::Affine3f base_T_cam_; // e.g. base_T_cam_ * cam_something = something in base frame

    float groundClearance_;
    float fieldRotation_; // Absolute rotation of the bot relative to the field
    // Field size from wall to the field middle
    float fieldSizeX_;
    float fieldSizeY_;

    std::deque<ExtFieldInfo> fieldInfo_;

    std::shared_ptr<TimeSync> pTimeSync_;
};
