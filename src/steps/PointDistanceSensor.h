#pragma once

#include "interface/FrameYUV420.h"
#include "util/Camera.h"
#include "util/TimeSync.h"
#include "interface/ProcessingStep.h"

class PointDistanceSensor : public AProcessingStep
{
public:
    PointDistanceSensor();

    void execute(State& state) override;

private:
    ExtPointDistanceSensorConfig cfg_;
};
