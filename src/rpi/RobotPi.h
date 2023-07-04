/*
 * RobotPi.h
 *
 *  Created on: 31.10.2020
 *      Author: AndreR
 */

#pragma once

#include "rpi/RPiCamera.h"
#include "StatGenerator.h"
#include "interface/ProcessingStep.h"

class RobotPi : public AProcessingStep
{
public:
    RobotPi();

    void addEnabledProcessingStep(std::unique_ptr<AProcessingStep> step);
    void addProcessingStep(std::unique_ptr<AProcessingStep> step);

    void run();

    void execute(State& state) override;

private:
    void frameCallback(FrameYUV420* pFrame);
    void timeCallback(uint32_t timestampUs);

    void onCameraConfig(ExtCameraConfig* pConfig);
    void onCameraControl(ExtCameraControl* pCtrl);
    void onShutdown(ExtShutdown* pShutdown);

    std::vector<std::unique_ptr<AProcessingStep>> steps_;

    State state_;

    rpi::RPiCamera rpicamera_;
    StatGenerator statGenerator_;
};
