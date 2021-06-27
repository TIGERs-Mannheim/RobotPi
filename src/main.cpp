/*
 * main.c
 *
 *  Created on: 23.08.2017
 *      Author: AndreR
 */

#include <detector/RegionsLocalisation.h>
#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "signal.h"

#include "detector/WallLocalisation.h"
#include "detector/GoalLocalisation.h"
#include "detector/MiniPreview.h"
#include "RobotPi.h"
#include "tests.h"

int main(int argc, const char **argv)
{
    bcm_host_init();
    vcos_init();

    RobotPi robotPi;
    robotPi.addDetector(std::make_shared<RegionsLocalisation>(robotPi.getTimeSync()));
    robotPi.addDetector(std::make_shared<GoalLocalisation>(robotPi.getTimeSync()));
    //robotPi.addDetector(std::make_shared<WallLocalisation>(robotPi.getTimeSync()));
    robotPi.addDetector(std::make_shared<MiniPreview>(2500000));
    robotPi.run();

//  TestCamResolutionChanges();
//  TestCamRecording();
//  TestCamImageCapture();
//  TestSerialConnection();

    return 0;
}
