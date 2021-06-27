/*
 * RobotPi.h
 *
 *  Created on: 31.10.2020
 *      Author: AndreR
 */

#pragma once

#include "rpi/RPiCamera.h"
#include "rpi/TigerComm.h"
#include "interface/Detector.h"
#include "Eigen/Dense"
#include "util/TimeSync.h"

class RobotPi
{
public:
    RobotPi();

    void addDetector(std::shared_ptr<Detector> detector);

    void run();

    std::shared_ptr<TimeSync> getTimeSync() const { return pTimeSync_; }

private:
    void handleMessage(std::shared_ptr<Command> pCmd);
    void frameCallback(FrameYUV420* pFrame);
    void timeCallback(uint32_t timestampUs);
    void sendStats();

    std::vector<std::shared_ptr<Detector>> detectors_;

    rpi::RPiCamera camera_;
    rpi::TigerComm comm_;

    uint64_t lastFrameTime_;

    Eigen::VectorXf dtSamples_;
    Eigen::VectorXf rtSamples_;
    int sampleIndex_;

    std::shared_ptr<TimeSync> pTimeSync_;
};
