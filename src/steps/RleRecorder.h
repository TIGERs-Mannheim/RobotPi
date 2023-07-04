#pragma once

#include "interface/ProcessingStep.h"
#include "util/UDPSocket.h"
#include "util/RLEFile.h"

class RLERecorder : public AProcessingStep
{
public:
    RLERecorder();

    void execute(State& state) override;

private:
    UDPSocket socket_ = UDPSocket("224.0.23.182", 28575);
    RLEFile recording_ = RLEFile::openNext();
};
