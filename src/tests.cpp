/*
 * tests.cpp
 *
 *  Created on: 31.10.2020
 *      Author: AndreR
 */

#include "rpi/RPiCamera.h"
#include "rpi/TigerComm.h"
#include "tests.h"

#include "util/log.h"
#include "Eigen/Dense"
#include "commands.h"
#include "version.h"

uint64_t timeLast_ = vcos_getmicrosecs64();
Eigen::VectorXf timeSamples_(50);
int timeSampleIndex_;

static void frameCallback(FrameYUV420 *pFrame)
{
    uint64_t timeNow = vcos_getmicrosecs64();
    float dt = (timeNow - timeLast_) * 1e-6f;
    timeLast_ = timeNow;

    timeSamples_[timeSampleIndex_++] = dt;

    if(timeSampleIndex_ == timeSamples_.rows())
    {
        timeSampleIndex_ = 0;

        float min = timeSamples_.minCoeff();
        float max = timeSamples_.maxCoeff();
        float meanDt = timeSamples_.mean();
        float meanDev = (timeSamples_.array() - meanDt).mean();

        fprintf(stderr, "FPS: %3.3f, dt: %.6f, dev: %.6f, min: %.6f, max: %.6f\n", 1.0f / meanDt, meanDt, meanDev, min, max);
    }
}

void TestCamResolutionChanges()
{
    rpi::RPiCamera camera;
    camera.setFrameCallback(&frameCallback);

    camera.start(rpi::ECameraRes::RES_640x480_FPS_90);
    vcos_sleep(3000);

    LogInfo("Switching to 1280x960 @42fps\n");
    camera.start(rpi::ECameraRes::RES_1280x960_FPS_42);
    vcos_sleep(3000);

    LogInfo("Switching to 2560x1920 @15fps\n");
    camera.start(rpi::ECameraRes::RES_2560x1920_FPS_15);
    vcos_sleep(3000);

    LogInfo("Switching to 1280x960 @42fps\n");
    camera.start(rpi::ECameraRes::RES_1280x960_FPS_42);
    vcos_sleep(3000);

    LogInfo("Switching to 640x480 @90fps\n");
    camera.start(rpi::ECameraRes::RES_640x480_FPS_90);
    vcos_sleep(3000);
}

void TestCamRecording()
{
    rpi::RPiCamera camera;
    camera.setFrameCallback(&frameCallback);

    LogInfo("Starting at 640x480 @90fps\n");
    camera.start(rpi::ECameraRes::RES_640x480_FPS_90);
    vcos_sleep(2000);

    camera.setRecording(true);
    vcos_sleep(3000);

    LogInfo("Trying to change resolution during recording, this will fail\n");
    camera.start(rpi::ECameraRes::RES_640x480_FPS_90);
    vcos_sleep(2000);

    camera.setRecording(false);
    vcos_sleep(2000);
}

void TestCamImageCapture()
{
    rpi::RPiCamera camera;
    camera.setFrameCallback(&frameCallback);

    LogInfo("Switching to 1280x960 @42fps\n");
    camera.start(rpi::ECameraRes::RES_1280x960_FPS_42);
    vcos_sleep(2000);

    LogInfo("Triggering image capture\n");
    camera.triggerCapture();
    vcos_sleep(2000);
}

void TestSerialConnection()
{
    using namespace std;

    shared_ptr<Command> pMsg;

    // create a communication object
    rpi::TigerComm comm;

    // open a serial port, user needs to be root or in dialout group
    if(!comm.open("/dev/ttyAMA0", 921600))
    {
        LogError("Cannot open serial port");
        return;
    }

    // A serial connection is now open, receiver works asynchronously on a different thread.
    // Received messages are buffered in a queue.

    // main loop for testing, run 5s
    for(uint32_t t = 0; t < 500; t++)
    {
        // process all queued messages from robot
        while((pMsg = comm.read()) != nullptr)
        {
            if(pMsg->getId() == CMD_SYSTEM_MATCH_FEEDBACK)
            {
                // cast to a real feedback structure, can be nullptr if data length is too short
                const SystemMatchFeedback* pFeedback = pMsg->as<SystemMatchFeedback>();

                if(pFeedback)
                {
                    cout << "Battery level: " << pFeedback->batteryLevel << "mV\n";
                }
            }

            if(pMsg->getId() == CMD_EXT_SHUTDOWN)
            {
                const ExtShutdown* pShutdown = pMsg->as<ExtShutdown>();

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
        }

        // send version information to micro controller
        static const char* buildTime = VERSION_BUILD_STRING;

        ExtRobotPiVersion version { 0 };
        version.major = VERSION_MAJOR;
        version.minor = VERSION_MINOR;
        memcpy(version.date, buildTime, sizeof(version.date));

        comm.write(CMD_EXT_ROBOT_PI_VERSION, version);

        // sleep 10ms to not annoy CPU
        vcos_sleep(10);
    }
}
