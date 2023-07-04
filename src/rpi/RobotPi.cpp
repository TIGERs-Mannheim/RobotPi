/*
 * RobotPi.cpp
 *
 *  Created on: 31.10.2020
 *      Author: AndreR
 */

#include "RobotPi.h"
#include "util/log.h"
#include "git_version.h"
#include "commands.h"
#include "TigerComm.h"
#include <filesystem>
#include <fstream>

using namespace std;

RobotPi::RobotPi()
{
    addCommandHandler<ExtCameraConfig>(CMD_EXT_CAMERA_CONFIG, std::bind(&RobotPi::onCameraConfig, this, std::placeholders::_1));
    addCommandHandler<ExtCameraControl>(CMD_EXT_CAMERA_CONTROL, std::bind(&RobotPi::onCameraControl, this, std::placeholders::_1));
    addCommandHandler(CMD_EXT_CAMERA_TRIGGER_CAPTURE, std::bind(&rpi::RPiCamera::triggerCapture, &rpicamera_));
    addCommandHandler<ExtShutdown>(CMD_EXT_SHUTDOWN, std::bind(&RobotPi::onShutdown, this, std::placeholders::_1));

    rpicamera_.setFrameCallback([&] (FrameYUV420* frame) { this->frameCallback(frame); });
    state_.pComm = std::make_unique<rpi::TigerComm>("/dev/ttyAMA0", 2500000, [&] (uint32_t time) { this->timeCallback(time); });
    state_.camera = Camera(CAMERA_CALIBRATION_FILE_PATH);
    state_.pComm->write({CMD_EXT_CAMERA_CALIBRATION, state_.camera.getCalibration()});
}

void RobotPi::run()
{
    rpicamera_.start(rpi::ECameraRes::RES_1280x960_FPS_42);

    while(true)
    {
        // send version information to micro controller
        ExtRobotPiVersion version { 0 };
        version.version = (GIT_VERSION_MAJOR << 24) | (GIT_VERSION_MINOR << 16) | (GIT_VERSION_PATCH << 8) | GIT_IS_DIRTY;
        version.gitRef = GIT_SHA1_SHORT;
        strncpy(version.date, GIT_COMMIT_DATE_ISO8601, sizeof(version.date));

        state_.pComm->write(Command(CMD_EXT_ROBOT_PI_VERSION, version));

        // Send update progress if available
        ExtUpdateProgress updateProgress;
        std::memset(&updateProgress, 0, sizeof(updateProgress));
        const std::string statusFile("/var/run/update_state");
        if(std::filesystem::exists(statusFile))
        {
            updateProgress.updateInProgress = 1;
            std::ifstream file(statusFile);
            std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            std::strncpy(updateProgress.status, content.c_str(), sizeof(updateProgress.status)-1);
        }

        state_.pComm->write(Command(CMD_EXT_UPDATE_PROGRESS, updateProgress));

        vcos_sleep(500);
    }
}

void RobotPi::timeCallback(uint32_t timestampUs)
{
    // called from TigerComm receiver thread!
    const int64_t stcTimestampI64 = rpicamera_.getStcTimestampUs();

    state_.timeSync.addSample(timestampUs, stcTimestampI64);
}

void RobotPi::frameCallback(FrameYUV420* pFrame)
{
    state_.pFrame = pFrame;
    const FrameMetadata& metadata = pFrame->getMetadata();
    state_.uvWidth = metadata.width/2;
    state_.uvHeight = metadata.height/2;

    execute(state_);

    // process all queued messages from robot
    shared_ptr<Command> pMsg;
    while((pMsg = state_.pComm->read()) != nullptr)
    {
        handleCommand(*pMsg);

        for(auto&& step : steps_)
        {
            step->handleCommand(*pMsg);
        }
    }
}

void RobotPi::execute(State &state)
{
    statGenerator_.frameStart();
    try
    {
        for(auto&& step : steps_)
        {
            if(step->isEnabled())
                step->execute(state_);
        }
    }
    catch(std::exception& ex)
    {
        std::cerr << "Frame processing exception: " << ex.what() << std::endl;
    }

    statGenerator_.frameEnd(*state_.pComm, rpicamera_);
}

void RobotPi::addEnabledProcessingStep(std::unique_ptr<AProcessingStep> step)
{
    step->setEnabled(true);
    addProcessingStep(std::move(step));
}

void RobotPi::addProcessingStep(std::unique_ptr<AProcessingStep> step)
{
    steps_.push_back(std::move(step));
}


void RobotPi::onCameraConfig(ExtCameraConfig* pConfig)
{
    rpicamera_.setExposureSettings({(bool)pConfig->expAutoMode, pConfig->expAnalogGain, pConfig->expDigitalGain, pConfig->expTimeUs});
    rpicamera_.setWhiteBalanceSettings({(bool)pConfig->wbAutoMode, pConfig->wbRedGain, pConfig->wbBlueGain});
}

void RobotPi::onCameraControl(ExtCameraControl* pCtrl)
{
    if(pCtrl->recording)
    {
        rpicamera_.setRecording(true);
    }
    else
    {
        rpicamera_.setRecording(false);

        if(pCtrl->resolution == 0)
            rpicamera_.start(rpi::ECameraRes::RES_640x480_FPS_90);
        else if(pCtrl->resolution == 2)
            rpicamera_.start(rpi::ECameraRes::RES_2560x1920_FPS_15);
        else
            rpicamera_.start(rpi::ECameraRes::RES_1280x960_FPS_42);
    }
}

void RobotPi::onShutdown(ExtShutdown* pShutdown)
{
    if(pShutdown->key == EXT_SHUTDOWN_KEY)
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
