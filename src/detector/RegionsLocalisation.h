/*
 * ColorClassTest.h
 *
 *  Created on: 22.11.2020
 *      Author: AndreR
 */

#pragma once

#include "util/Camera.h"
#include "interface/Detector.h"
#include "util/ColorClassifierYUV.h"
#include "util/CMVision.h"
#include "util/Circle.h"
#include "util/TimeSync.h"
#include "commands.h"
#include "util/BallTracker.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class RegionsLocalisation : public Detector
{
public:
    RegionsLocalisation(std::shared_ptr<TimeSync> pTimeSync);

    Command::List processFrame(FrameYUV420* pFrame) override;
    void processCommand(std::shared_ptr<Command> pCmd) override;
    void setDebugLevel(uint8_t level) override { debugLevel_ = level; }

private:
    struct DetectedBall
    {
        bool valid;
        Eigen::Vector2i centroid; // luma resolution
        int32_t area; // pixel in chroma resolution
        CircleFit circleFit; // chroma resolution
        Eigen::Vector3f posFromDiameter_base;
        Eigen::Vector3f posFromPlaneIntersection_base;
        Eigen::Vector3f posFromDiameter_map;
        Eigen::Vector3f posFromPlaneIntersection_map;
    };

    DetectedBall processRegion(FrameYUV420* pFrame, const ColorClassifierYUV::Output* pClassified, const ColoredRegion& region);

    ExtRobotState updateTransforms(uint32_t ucTime, FrameYUV420* pFrame);
    Eigen::Vector2f mapPosToDistortedImagePos(const Eigen::Vector3f& pos_map);
    void debugDrawBalls(const std::vector<RegionsLocalisation::DetectedBall>& balls, FrameYUV420* pFrame);
    void debugDrawTrackers(FrameYUV420* pFrame);

    uint8_t debugLevel_;
    ColorClassifierYUV classifier_;
    CMVision cmVision_;
    Camera camera_;

    Eigen::Vector3f base_t_cam_;
    float camRotation_;
    float groundClearance_;

    Eigen::Affine3f base_T_cam_; // e.g. base_T_cam_ * cam_something = something in base frame
    Eigen::Affine3f map_T_base_; // e.g. map_T_base_ * base_something = something in map frame
    Eigen::Affine3f cam_T_base_;
    Eigen::Affine3f base_T_map_;

    std::shared_ptr<TimeSync> pTimeSync_;

    std::deque<ExtRobotState> robotStates_;

    std::vector<BallTracker> trackers_;
    uint8_t nextTrackerId_;

    ExtBallDetectionConfig detectionConfig_;
};
