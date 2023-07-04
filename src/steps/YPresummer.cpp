#include "YPresummer.h"

YPresummer::YPresummer()
{
    addDefaultStepConfigHandler(EXT_STEP_MASK_YPRESUMMER);
}

void YPresummer::execute(State& state)
{
    presummer_.setOutputSize(state.uvWidth, state.uvHeight);
    presummer_.presum(state.threadPool, state.pFrame->getYData(), state.pFrame->getMetadata().width, state.pFrame->getMetadata().height);
    state.pFrameYDataUVSized = presummer_.getOutput();
}
