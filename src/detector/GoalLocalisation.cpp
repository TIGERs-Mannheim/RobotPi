/*
 * WallLocalisation.cpp
 *
 *  Created on: 22.05.2021
 *      Author: FelixW
 */

#include "GoalLocalisation.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <bits/stdc++.h>


GoalLocalisation::GoalLocalisation(std::shared_ptr<TimeSync> pTimeSync):
 debugLevel_(1),
 pTimeSync_(pTimeSync)
{
}

void GoalLocalisation::processCommand(std::shared_ptr<Command> pCmd)
{
    switch(pCmd->getId())
    {
        case CMD_EXT_CAMERA_INTRINSICS:
        {
            const ExtCameraIntrinsics* pIntrinsics = pCmd->as<ExtCameraIntrinsics>();
            if(pIntrinsics)
            {
                camera_.setIntrinsics(*pIntrinsics);
            }
        }
        break;
    }
}

static bool compareGoalSizes(const ExtGoalDetections::_goals& goal1, const ExtGoalDetections::_goals& goal2)
{
    return (goal1.rightAngle - goal1.leftAngle) > (goal2.rightAngle - goal2.leftAngle);
}

Command::List GoalLocalisation::processFrame ( FrameYUV420* pFrame )
{
    const uint8_t whiteDecider = 128;
    const double minGoalSize = EIGEN_PI / 64;
    
    const uint32_t width = pFrame->getMetadata().width;
    const uint32_t height = pFrame->getMetadata().height;
    const uint8_t* yData = pFrame->getYData();
    
    camera_.setCurrentResolutionWidth(width);
    
    // Find the goal(s)
    uint32_t scanlineY = (uint32_t)(height/2 + tan(-15.25 * EIGEN_PI / 180.0) * camera_.getFocalLength());
    bool inGoal = false;
    Eigen::Vector2f goalStart;
    std::vector<ExtGoalDetections::_goals> goals;
    for(uint32_t x = 0; x < width; x++)
    {
        Eigen::Vector2f pos = camera_.distort(Eigen::Vector2f(x, scanlineY));
        bool newInGoal = yData[(int32_t)pos.y()*width + (int32_t)pos.x()] > whiteDecider;

        if(newInGoal && !inGoal)
        {
            goalStart = pos;
        }
        else if(!newInGoal && inGoal)
        {
            float leftAngle = atan2(camera_.undistort(goalStart).x() - width/2, camera_.getFocalLength());
            float rightAngle = atan2(camera_.undistort(pos).x() - width/2, camera_.getFocalLength());
            if(rightAngle - leftAngle > minGoalSize)
            {
                if(leftAngle < 0.0 && rightAngle > 0.0) // Filter to only center goal
                {
                    goals.push_back({leftAngle, rightAngle});
                }
            }
        }

        inGoal = newInGoal;
    }
    sort(goals.begin(), goals.end(), compareGoalSizes);

    // Draw recognized goals on the frame, print debug info
    if(debugLevel_)
    {
        for(const auto& goal : goals)
        {
            if(debugLevel_ > 1)
                std::cout << "Goal: " << goal.leftAngle << " : " << goal.rightAngle << std::endl;

            Eigen::Vector2f start = camera_.distort(Eigen::Vector2f(tan(goal.leftAngle) * camera_.getFocalLength() + width/2, scanlineY));
            Eigen::Vector2f end = camera_.distort(Eigen::Vector2f(tan(goal.rightAngle) * camera_.getFocalLength() + width/2, scanlineY));
            pFrame->drawFilledRect(start.x(), start.y(), end.x(), end.y() + 40.0, ColorYUV(64, 255, 127));
        }
    }

    // Construct commands
    Command::List commands;
    const uint32_t ucTime = pTimeSync_->stc2mc(pFrame->getMetadata().timestampUs - pFrame->getMetadata().exposureUs);

    ExtGoalDetections goalCommand;
    goalCommand.timestampUs = ucTime;
    goalCommand.numGoals = (uint8_t)std::min(EXT_GOAL_DETECTIONS_MAX_GOALS, (int)goals.size());
    for(uint8_t i = 0; i < goalCommand.numGoals; i++)
    {
        goalCommand.goals[i] = goals[i];
    }
    commands.push_back(Command(CMD_EXT_GOAL_DETECTIONS, goalCommand));

    return commands;
}
