#pragma once

#include "Eigen/Core"
#include "RPiCamera.h"
#include "interface/CommandTransceiver.h"

class StatGenerator
{
public:
    StatGenerator();

    void frameStart();
    void frameEnd(CommandTransceiver& comm, rpi::RPiCamera& rpicamera);

private:
    uint64_t lastFrameTime_;
    Eigen::VectorXf dtSamples_;
    Eigen::VectorXf rtSamples_;
    int sampleIndex_;
};
