#pragma once

#include "interface/ProcessingStep.h"


class CameraCalibration : public AProcessingStep
{
public:
    CameraCalibration();

    void execute(State& state) override;

private:
    bool hasCalibrated_ = false;
};
