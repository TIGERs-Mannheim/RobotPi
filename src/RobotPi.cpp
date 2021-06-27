/*
 * RobotPi.cpp
 *
 *  Created on: 31.10.2020
 *      Author: AndreR
 */

#include "RobotPi.h"
#include "util/log.h"
#include "version.h"
#include "commands.h"
#include <numeric>

using namespace std;

RobotPi::RobotPi()
:dtSamples_(50), rtSamples_(50), sampleIndex_(0)
{
    pTimeSync_ = std::make_shared<TimeSync>();

    camera_.setFrameCallback(bind(&RobotPi::frameCallback, this, placeholders::_1));
    comm_.setRemoteTimeCallback(bind(&RobotPi::timeCallback, this, placeholders::_1));

    lastFrameTime_ = vcos_getmicrosecs64();
}

void RobotPi::addDetector(std::shared_ptr<Detector> detector)
{
    detectors_.push_back(detector);
}

void RobotPi::run()
{
    if(!comm_.open("/dev/ttyAMA0", 2500000))
    {
        LogError("Cannot open serial port");
        return;
    }

    camera_.start(rpi::ECameraRes::RES_1280x960_FPS_42);

    while(true)
    {
        // send version information to micro controller
        static const char* buildTime = VERSION_BUILD_STRING;

        ExtRobotPiVersion version { 0 };
        version.major = VERSION_MAJOR;
        version.minor = VERSION_MINOR;
        memcpy(version.date, buildTime, sizeof(version.date));

        comm_.write(CMD_EXT_ROBOT_PI_VERSION, version);

        vcos_sleep(500);
    }
}

void RobotPi::timeCallback(uint32_t timestampUs)
{
    // called from TigerComm receiver thread!
    const int64_t stcTimestampI64 = camera_.getStcTimestampUs();

    pTimeSync_->addSample(timestampUs, stcTimestampI64);
}

void RobotPi::frameCallback(FrameYUV420* pFrame)
{
    uint64_t timeNow = vcos_getmicrosecs64();
    float dt = (timeNow - lastFrameTime_) * 1e-6f;
    lastFrameTime_ = timeNow;

    dtSamples_[sampleIndex_] = dt;

    // process algorithms
    for(auto&& detector : detectors_)
    {
        Command::List output = detector->processFrame(pFrame);

        for(auto&& cmd : output)
        {
            comm_.write(cmd);
        }
    }

    // process all queued messages from robot
    shared_ptr<Command> pMsg;
    while((pMsg = comm_.read()) != nullptr)
    {
        handleMessage(pMsg);

        for(auto&& detector : detectors_)
        {
            detector->processCommand(pMsg);
        }
    }

    uint64_t timeEnd = vcos_getmicrosecs64();
    float rt = (timeEnd - timeNow) * 1e-6f;

    rtSamples_[sampleIndex_] = rt;
    ++sampleIndex_;

    if(sampleIndex_ == dtSamples_.rows())
    {
        sampleIndex_ = 0;

        sendStats();
    }
}

void RobotPi::sendStats()
{
    ExtCameraStats stats;
    stats.width = camera_.getResolution().width;
    stats.height = camera_.getResolution().height;

    stats.dtMin = dtSamples_.minCoeff();
    stats.dtMax = dtSamples_.maxCoeff();
    stats.dtAvg = dtSamples_.mean();
    stats.dtDev = sqrtf((dtSamples_.array() - stats.dtAvg).square().mean());

    stats.rtMin = rtSamples_.minCoeff();
    stats.rtMax = rtSamples_.maxCoeff();
    stats.rtAvg = rtSamples_.mean();
    stats.rtDev = sqrtf((rtSamples_.array() - stats.rtAvg).square().mean());

    stats.recording = camera_.isRecording();
    memcpy(stats.recordFilename, camera_.getRecordFilename(), sizeof(stats.recordFilename));
    stats.recordDuration = camera_.getRecordDuration();
    stats.recordSize = camera_.getRecordSize();

    stats.imagesTaken = camera_.getImagesTaken();
    memcpy(stats.imageFilename, camera_.getImageFilename(), sizeof(stats.imageFilename));

    comm_.write(CMD_EXT_CAMERA_STATS, stats);
}

void RobotPi::handleMessage(std::shared_ptr<Command> pCmd)
{
    switch(pCmd->getId())
    {
        case CMD_EXT_CAMERA_CONFIG:
        {
            const ExtCameraConfig* pConfig = pCmd->as<ExtCameraConfig>();

            if(pConfig)
            {
                rpi::ExposureSettings exp;
                exp.autoMode = pConfig->expAutoMode;
                exp.analogGain = pConfig->expAnalogGain;
                exp.digitalGain = pConfig->expDigitalGain;
                exp.exposureTimeUs = pConfig->expTimeUs;

                camera_.setExposureSettings(exp);

                rpi::WhiteBalanceSettings wb;
                wb.autoMode = pConfig->wbAutoMode;
                wb.redGain = pConfig->wbRedGain;
                wb.blueGain = pConfig->wbBlueGain;

                camera_.setWhiteBalanceSettings(wb);
            }
        }
        break;
        case CMD_EXT_SHUTDOWN:
        {
            const ExtShutdown* pShutdown = pCmd->as<ExtShutdown>();

            if(pShutdown && pShutdown->key == EXT_SHUTDOWN_KEY)
            {
                LogInfo("Valid shutdown received, terminating\n");

                // requires super-user privileges, either run as root or use sudo
                int result = system("shutdown -h now");
                if(result < 0)
                {
                    LogError("Shutdown failed, error: %d\n", result);
                }
            }
        }
        break;
        case CMD_EXT_CAMERA_CONTROL:
        {
            const ExtCameraControl* pCtrl = pCmd->as<ExtCameraControl>();
            if(pCtrl)
            {
                if(pCtrl->recording)
                {
                    camera_.setRecording(true);
                }
                else
                {
                    camera_.setRecording(false);

                    if(pCtrl->resolution == 0)
                        camera_.start(rpi::ECameraRes::RES_640x480_FPS_90);
                    else if(pCtrl->resolution == 2)
                        camera_.start(rpi::ECameraRes::RES_2560x1920_FPS_15);
                    else
                        camera_.start(rpi::ECameraRes::RES_1280x960_FPS_42);
                }
            }
        }
        break;
        case CMD_EXT_CAMERA_TRIGGER_CAPTURE:
        {
            // There is no data structure for this command
            camera_.triggerCapture();
        }
        break;
        default:
            break;
    }
}
