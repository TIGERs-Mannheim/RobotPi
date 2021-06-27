/*
 * test_main.cpp
 *
 *  Created on: 01.02.2021
 *      Author: FelixW
 */
#include <iostream>
#include "cvutils.h"
#include "detector/WallLocalisation.h"
#include "detector/RegionsLocalisation.h"
#include "detector/GoalLocalisation.h"
#include <chrono>

int main(int argc, const char **argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: robotpi [path/to/test_image]" << std::endl;
        return 0;
    }

    FrameYUV420 yuvFrame = imageByPath(argv[1]);

    GoalLocalisation detector(std::make_shared<TimeSync>());
    auto startTime = std::chrono::high_resolution_clock::now();

    detector.processFrame(&yuvFrame);

    auto stopTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
    std::cout << duration.count() << std::endl;

    showFrame(yuvFrame);

    return 0;
}
