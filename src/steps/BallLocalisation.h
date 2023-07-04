#pragma once

#include "util/Circle.h"
#include "util/TimeSync.h"
#include "commands.h"
#include "util/BallTracker.h"
#include "interface/ProcessingStep.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class BallLocalisation : public AProcessingStep
{
public:
    BallLocalisation();

    void execute(State& state) override;

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

    DetectedBall processRegion(State& state, const ColoredRegion& region);

    ExtRobotState updateTransforms(uint32_t ucTime, FrameYUV420* pFrame);
    Eigen::Vector2f mapPosToDistortedImagePos(const Camera& camera, const Eigen::Vector3f& pos_map);
    void debugDrawBalls(const std::vector<BallLocalisation::DetectedBall>& balls, FrameYUV420* pFrame);
    void debugDrawTrackers(const Camera& camera, FrameYUV420* pFrame);

    Eigen::Affine3f map_T_base_; // e.g. map_T_base_ * base_something = something in map frame
    Eigen::Affine3f base_T_map_;

    std::deque<ExtRobotState> robotStates_;

    std::vector<BallTracker> trackers_;
    uint8_t nextTrackerId_;

    ExtBallLocalisationConfig config_;

    static constexpr float BALL_DIAMETER = 0.043f;
};
