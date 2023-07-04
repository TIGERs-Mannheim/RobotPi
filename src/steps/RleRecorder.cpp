#include "RleRecorder.h"

RLERecorder::RLERecorder()
{
    addDefaultStepConfigHandler(EXT_STEP_MASK_RLE_RECORDER);
}

void RLERecorder::execute(State &state)
{
    RLEFrame frame(state.pFrame->getMetadata(), state.frameColoredRuns);
    recording_.append(frame);
    socket_.send(frame.getData());
}
