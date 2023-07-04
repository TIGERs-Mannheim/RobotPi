/*
 * main.c
 *
 *  Created on: 23.08.2017
 *      Author: AndreR
 */

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "RobotPi.h"
#include "tests.h"
#include "steps/YPresummer.h"
#include "steps/BallLocalisation.h"
#include "steps/PointDistanceSensor.h"
#include "steps/MiniPreview.h"
#include "steps/RleRecorder.h"
#include "steps/CameraCalibration.h"

int main(int argc, const char **argv)
{
    bcm_host_init();
    vcos_init();

    RobotPi robotPi;

    robotPi.addEnabledProcessingStep(std::make_unique<YPresummer>());
    robotPi.addEnabledProcessingStep(std::make_unique<ColorClassifierYUV>());
    robotPi.addEnabledProcessingStep(std::make_unique<RegionExtractor>());
    robotPi.addEnabledProcessingStep(std::make_unique<BallLocalisation>());
    robotPi.addProcessingStep(std::make_unique<PointDistanceSensor>());
    robotPi.addProcessingStep(std::make_unique<MiniPreview>(2500000));
    robotPi.addEnabledProcessingStep(std::make_unique<RLERecorder>());
    robotPi.addProcessingStep(std::make_unique<CameraCalibration>());

    robotPi.run();

//  TestCamResolutionChanges();
//  TestCamRecording();
//  TestCamImageCapture();
//  TestSerialConnection();

    return 0;
}
