#include <interface/vcos/vcos_thread.h>
#include "StatGenerator.h"
#include "commands.h"


StatGenerator::StatGenerator(): dtSamples_(50), rtSamples_(50), sampleIndex_(0)
{
    lastFrameTime_ = vcos_getmicrosecs64();
}

void StatGenerator::frameStart()
{
    uint64_t timeNow = vcos_getmicrosecs64();
    float dt = (timeNow - lastFrameTime_) * 1e-6f;
    lastFrameTime_ = timeNow;

    dtSamples_[sampleIndex_] = dt;
}

void StatGenerator::frameEnd(CommandTransceiver& comm, rpi::RPiCamera& rpicamera)
{
    uint64_t timeEnd = vcos_getmicrosecs64();
    float rt = (timeEnd - lastFrameTime_) * 1e-6f;

    rtSamples_[sampleIndex_] = rt;
    ++sampleIndex_;

    if(sampleIndex_ == dtSamples_.rows())
    {
        sampleIndex_ = 0;

        ExtCameraStats stats;
        stats.width = rpicamera.getResolution().width;
        stats.height = rpicamera.getResolution().height;

        stats.dtMin = dtSamples_.minCoeff();
        stats.dtMax = dtSamples_.maxCoeff();
        stats.dtAvg = dtSamples_.mean();
        stats.dtDev = sqrtf((dtSamples_.array() - stats.dtAvg).square().mean());

        stats.rtMin = rtSamples_.minCoeff();
        stats.rtMax = rtSamples_.maxCoeff();
        stats.rtAvg = rtSamples_.mean();
        stats.rtDev = sqrtf((rtSamples_.array() - stats.rtAvg).square().mean());

        printf("dtAvg: %f rtAvg: %f\n", stats.dtAvg, stats.rtAvg);

        stats.recording = rpicamera.isRecording();
        memcpy(stats.recordFilename, rpicamera.getRecordFilename(), sizeof(stats.recordFilename));
        stats.recordDuration = rpicamera.getRecordDuration();
        stats.recordSize = rpicamera.getRecordSize();

        stats.imagesTaken = rpicamera.getImagesTaken();
        memcpy(stats.imageFilename, rpicamera.getImageFilename(), sizeof(stats.imageFilename));

        comm.write({CMD_EXT_CAMERA_STATS, stats});
    }
}
