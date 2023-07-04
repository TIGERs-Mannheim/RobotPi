#pragma once

#include "interface/ProcessingStep.h"
#include "util/Presummer.h"

class YPresummer : public AProcessingStep
{
public:
    YPresummer();

    void execute(State& state) override;

private:
    Presummer presummer_ = Presummer(640, 480);
};
