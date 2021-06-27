/*
 * ColorClassTest.cpp
 *
 *  Created on: 22.11.2020
 *      Author: AndreR
 */

/**
>> cameraParams2.IntrinsicMatrix
          1345.66206049234                         0                         0
                         0          1346.67286824137                         0
          1305.39118903478          971.448626959762                         1

>> cameraParams3.IntrinsicMatrix
           1342.1979748795                         0                         0
                         0          1340.36391825491                         0
           1311.6673564072          979.247312117014                         1

>> cameraParams2.RadialDistortion (err 1.9pix)
        -0.279324740320405        0.0581955384718963

>> cameraParams3.RadialDistortion (err 1.0pix)
        -0.325825805824401         0.119036424008428       -0.0208066507388343

   Pixel size: 1.4µm x 1.4µm
*/

#include "detector/RegionsLocalisation.h"
#include "util/log.h"
#include <iomanip>
#include "util/KalmanFilter.h"
#include "util/TrackingFilterPosVel.h"
#include <iostream>

RegionsLocalisation::RegionsLocalisation(std::shared_ptr<TimeSync> pTimeSync)
:debugLevel_(3),
 camRotation_(-90.0f*M_PI/180.0f),
 groundClearance_(0.0f),
 pTimeSync_(pTimeSync),
 nextTrackerId_(0)
{
    detectionConfig_.ballDiameter = 0.043f;
    detectionConfig_.greedyCircleFactor = 1.25f;
    detectionConfig_.maxTrackers = 5;
    detectionConfig_.minBlobArea = 5;
    detectionConfig_.topImageSkipFactor = 0.25f;
    detectionConfig_.trackerTimeout = 1.0f;

    classifier_.updateThreshold(Color::Orange, { 0, 255, 0, 160, 150, 255 });
    classifier_.updateThreshold(Color::White, { 96, 255, 128-16, 128+32, 128-48, 128 });
}

Command::List RegionsLocalisation::processFrame(FrameYUV420* pFrame)
{
    Command::List commands;

    // Camera model at chroma channel resolution
    camera_.setCurrentResolutionWidth(pFrame->getMetadata().width/2);

    // get robot timestamp at image time and update transforms
    const uint32_t ucTime = pTimeSync_->stc2mc(pFrame->getMetadata().timestampUs - pFrame->getMetadata().exposureUs);
    ExtRobotState usedRobotState = updateTransforms(ucTime, pFrame);

    // classify colors in image
    ColorClassifierYUV::Output classifiedImage = classifier_.process(pFrame);

    // extract regions from classified image
    const CMVision::ColorRegionLUTPtr pColorRegionMap =
        cmVision_.extractRegions(classifiedImage.pColorClass,
                                 classifiedImage.width, classifiedImage.height, detectionConfig_.minBlobArea);

    // process orange regions (balls?)
    const auto& orangeList = (*pColorRegionMap)[1 << static_cast<int>(Color::Orange)];

    std::vector<RegionsLocalisation::DetectedBall> balls;
    int32_t topSkipRows = pFrame->getMetadata().height * detectionConfig_.topImageSkipFactor;

    for(const auto& reg : orangeList)
    {
        if(reg.y1 <= topSkipRows/2)
            continue;

        auto ball = processRegion(pFrame, &classifiedImage, reg);
        if(ball.valid)
        {
            balls.push_back(ball);
        }
    }

    // sort by size of ball
    std::sort(balls.begin(), balls.end(), [](const RegionsLocalisation::DetectedBall& a, const RegionsLocalisation::DetectedBall& b) {
       return a.circleFit.getCircle().radius() > b.circleFit.getCircle().radius();
    });

    // filter balls which overlay with a fitted circle (it's probably noise at edges), go from large to small
    auto iter = balls.begin();
    while(iter != balls.end())
    {
        const auto& ballA = *iter;
        Circle circle1 = ballA.circleFit.getCircle();
        Eigen::Vector2f center1(circle1.x(), circle1.y());

        auto checkIter = iter+1;
        while(checkIter != balls.end())
        {
            const auto& ballB = *checkIter;
            Eigen::Vector2f center2 = ballB.centroid.cast<float>()*0.5f;

            if((center1 - center2).norm() < circle1.radius()*detectionConfig_.greedyCircleFactor)
            {
                checkIter = balls.erase(checkIter);
            }
            else
            {
                ++checkIter;
            }
        }

        ++iter;
    }

    const double tFrameStc = pFrame->getMetadata().timestampUs * 1e-6;

    // remove trackers not updated for some time
    trackers_.erase(std::remove_if(trackers_.begin(), trackers_.end(),
        [=](const auto& t){
        return (tFrameStc - t.getLastCaptureTimestamp()) > detectionConfig_.trackerTimeout; }),
        trackers_.end());

    // do a prediction on all trackers
    for(auto& tracker : trackers_)
        tracker.predict(tFrameStc);

    // iterate over all balls
    for(size_t i = 0; i < balls.size() && i < detectionConfig_.maxTrackers; i++)
    {
        const auto& ball = balls[i];
        bool consumed = false;

        // offer ball to all trackers
        for(auto& tracker : trackers_)
        {
            if(tracker.update(tFrameStc, ball.posFromPlaneIntersection_map.cast<double>()))
            {
                consumed = true;
                break;
            }
        }

        // create new tracker if nobody wanted this ball
        if(!consumed && trackers_.size() < detectionConfig_.maxTrackers)
        {
            BallTracker tracker(nextTrackerId_, ball.posFromPlaneIntersection_map.cast<double>(), tFrameStc);
            trackers_.push_back(tracker);
            nextTrackerId_++;
        }
    }

    std::sort(trackers_.begin(), trackers_.end(), [&](const BallTracker& a, const BallTracker& b) {
       return (base_T_map_ * a.getPosition().cast<float>()).norm() < (base_T_map_ * b.getPosition().cast<float>()).norm();
    });

    // draw debug information
    if(debugLevel_ > 0)
    {
        if(detectionConfig_.topImageSkipFactor > 0.0f)
        {
            pFrame->drawLine(0, topSkipRows, pFrame->getMetadata().width, topSkipRows, ColorRGB(0.3f, 1.0f, 0.3f));
        }
    }

    if(debugLevel_ > 1)
        debugDrawBalls(balls, pFrame);

    if(debugLevel_ > 2)
        debugDrawTrackers(pFrame);

    // create detection message
    ExtBallDetections detections;
    detections.timestampUs = ucTime;
    memcpy(detections.robotPos, usedRobotState.posGlobal, sizeof(float)*3);

    uint32_t numBalls = trackers_.size();
    if(numBalls > EXT_BALL_DETECTIONS_MAX_BALLS)
        numBalls = EXT_BALL_DETECTIONS_MAX_BALLS;

    detections.numBalls = numBalls;

    for(uint32_t i = 0; i < numBalls; i++)
    {
        const auto map_ballPos  = trackers_[i].getPosition();
        const auto map_ballVel  = trackers_[i].getVelocity();

        detections.balls[i].trackerId = trackers_[i].getId();
        detections.balls[i].pos[0] = map_ballPos.x();
        detections.balls[i].pos[1] = map_ballPos.y();
        detections.balls[i].pos[2] = map_ballPos.z();
        detections.balls[i].vel[0] = map_ballVel.x();
        detections.balls[i].vel[1] = map_ballVel.y();
        detections.balls[i].vel[2] = map_ballVel.z();

        const auto line = trackers_[i].getLineFit();
        if(line.isValid())
        {
            detections.balls[i].linePos[0] = line.getLine().support().x();
            detections.balls[i].linePos[1] = line.getLine().support().y();
            detections.balls[i].lineDir[0] = line.getLine().direction().x();
            detections.balls[i].lineDir[1] = line.getLine().direction().y();
        }
        else
        {
            detections.balls[i].linePos[0] = NAN;
            detections.balls[i].linePos[1] = NAN;
            detections.balls[i].lineDir[0] = NAN;
            detections.balls[i].lineDir[1] = NAN;
        }
    }

    commands.push_back(Command(CMD_EXT_BALL_DETECTIONS, detections));

    return commands;
}

Eigen::Vector2f RegionsLocalisation::mapPosToDistortedImagePos(const Eigen::Vector3f& pos_map)
{
    Eigen::Vector3f pos_cam = cam_T_base_ * base_T_map_ * pos_map;
    if(pos_cam.z() < 0.001f)
        return Eigen::Vector2f(NAN, NAN);

    return camera_.distort(camera_.getImageCoordinates(pos_cam))*2;
}

ExtRobotState RegionsLocalisation::updateTransforms(uint32_t ucTime, FrameYUV420* pFrame)
{
    const auto& minIter = std::min_element(robotStates_.begin(), robotStates_.end(), [&](const ExtRobotState& a, const ExtRobotState& b) {
        return std::abs((int32_t)(a.timestampUs - ucTime)) < std::abs((int32_t)(b.timestampUs - ucTime));
    });

    ExtRobotState usedState;

    if(minIter != robotStates_.end())
    {
        usedState = *minIter;
        map_T_base_ = Eigen::Translation3f(minIter->posGlobal[0], minIter->posGlobal[1], 0)
            * Eigen::AngleAxisf(-M_PI/2.0f + minIter->posGlobal[2], Eigen::Vector3f::UnitZ());
        base_T_map_ = map_T_base_.inverse(Eigen::Affine);
    }
    else
    {
        map_T_base_.setIdentity();
        base_T_map_.setIdentity();
        memset(&usedState, 0, sizeof(ExtRobotState));
    }

    base_T_cam_ = Eigen::Translation3f(base_t_cam_) * Eigen::AngleAxisf(camRotation_, Eigen::Vector3f::UnitX());
    cam_T_base_ = base_T_cam_.inverse(Eigen::Affine);

    if(debugLevel_ > 0)
    {
        std::stringstream str;
        str << std::setprecision(4) << "map_pos: " << minIter->posGlobal[0] << ", "
            << minIter->posGlobal[1] << ", " << minIter->posGlobal[2];
        pFrame->drawText(5, 15, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);
    }

    return usedState;
}

RegionsLocalisation::DetectedBall RegionsLocalisation::processRegion(
    FrameYUV420* pFrame,
    const ColorClassifierYUV::Output* pClassified,
    const ColoredRegion& region)
{
    std::vector<Eigen::Vector2f> edges;

    // find edges of ball
    for(int32_t row = region.y1; row < region.y2; row++)
    {
        const uint8_t* pRow = pClassified->pColorClass + row*pClassified->width;

        if(region.x1 > 0)
        {
            // region not at left image edge

            for(int32_t col = region.x1; col < region.x2; col++)
            {
                if(pRow[col] == region.color)
                {
                    edges.push_back({col, row});
                    break;
                }
            }
        }

        if(region.x2 < (int32_t)pClassified->width-3)
        {
            // region not at right image edge:
            // note that -1 would work, but the RPi camera outputs a white strip on the right edge of the image
            // in 1280x960 mode, hence we use -3 to ignore this strip

            for(int32_t col = region.x2; col > region.x1; col--)
            {
                if(pRow[col] == region.color)
                {
                    edges.push_back({col, row});
                    break;
                }
            }
        }
    }

    // undistort edges' coordinates
    auto undistorted = camera_.undistort(edges);

    // draw debug edges
    if(debugLevel_ > 1)
    {
        for(const auto& edge : edges)
        {
            pFrame->drawPoint(edge.x()*2, edge.y()*2, ColorRGB(1.0f, 0.0f, 0.0f));
        }

        for(const auto& p : undistorted)
        {
            pFrame->drawPoint(p.x()*2, p.y()*2, ColorRGB(1.0f, 1.0f, 0.0f));
        }
    }

    // fit circle and determine 3D position
    RegionsLocalisation::DetectedBall detectedBall;
    detectedBall.valid = false;

    CircleFit fit(undistorted);
    if(fit.isValid() && fit.getResidualNormalized() < 1000)
    {
        Circle circle = fit.getCircle();

        detectedBall.valid = true;
        detectedBall.circleFit = fit;
        detectedBall.centroid = Eigen::Vector2i(region.cen_x*2, region.cen_y*2);
        detectedBall.area = region.area;

        const float focalLength = camera_.getFocalLength();

        // determine 3D pos from circle diameter
        float distance = detectionConfig_.ballDiameter * focalLength / (circle.radius()*2.0f); // Z plane distance, NOT distance to ball
        Eigen::Vector3f cam_ball = camera_.getNormalizedRay(Eigen::Vector2f(circle.x(), circle.y())) * distance;
        detectedBall.posFromDiameter_base = base_T_cam_ * cam_ball;
        detectedBall.posFromDiameter_map = map_T_base_ * detectedBall.posFromDiameter_base;

        // determine 3D pos from floor intersection (assuming ball is not airborne)
        Eigen::Vector3f cam_ray = camera_.getNormalizedRay(Eigen::Vector2f(circle.x(), circle.y())).normalized();
        auto base_ray = base_T_cam_.linear() * cam_ray;
        Eigen::ParametrizedLine<float, 3> cam_rayLine(base_T_cam_.translation(), base_ray);
        Eigen::Hyperplane<float, 3> floor(Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitZ()*(detectionConfig_.ballDiameter/2-groundClearance_));

        float intersectionParam = cam_rayLine.intersectionParameter(floor);
        if(intersectionParam > 0)
        {
            detectedBall.posFromPlaneIntersection_base = cam_rayLine.intersectionPoint(floor);
            detectedBall.posFromPlaneIntersection_map = map_T_base_ * detectedBall.posFromPlaneIntersection_base;
        }
        else
        {
            detectedBall.posFromPlaneIntersection_base.setConstant(NAN);
            detectedBall.posFromPlaneIntersection_map.setConstant(NAN);
        }
    }

    return detectedBall;
}

void RegionsLocalisation::processCommand(std::shared_ptr<Command> pCmd)
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
        case CMD_EXT_CAMERA_EXTRINSICS:
        {
            const ExtCameraExtrinsics* pExtrinsics = pCmd->as<ExtCameraExtrinsics>();
            if(pExtrinsics)
            {
                base_t_cam_ = Eigen::Vector3f(pExtrinsics->cameraPos[0], pExtrinsics->cameraPos[1], pExtrinsics->cameraPos[2]);
                camRotation_ = (-pExtrinsics->cameraTiltDeg - 90) * M_PI/180.0f;
                groundClearance_ = pExtrinsics->groundClearance;
            }
        }
        break;
        case CMD_EXT_ROBOT_STATE:
        {
            const ExtRobotState* pState = pCmd->as<ExtRobotState>();
            if(pState)
            {
                robotStates_.push_back(*pState);

                while(robotStates_.size() > 1000)
                    robotStates_.pop_front();
            }
        }
        break;
        case CMD_EXT_BALL_DETECTION_CONFIG:
        {
            const ExtBallDetectionConfig* pConfig = pCmd->as<ExtBallDetectionConfig>();
            if(pConfig)
            {
                detectionConfig_ = *pConfig;
            }
        }
        break;
        case CMD_EXT_BALL_TRACKER_CONFIG:
        {
            const ExtBallTrackerConfig* pConfig = pCmd->as<ExtBallTrackerConfig>();
            if(pConfig)
            {
                for(auto& tracker : trackers_)
                {
                    tracker.setHistorySize(pConfig->historySize);
                    tracker.setMaxVelocity(pConfig->maxVelocity);
                    tracker.setMeasurementError(pConfig->measError);
                    tracker.setModelError(pConfig->modelError);
                }
            }
        }
        break;
        case CMD_EXT_COLOR_THRESHOLDS:
        {
            const ExtColorThresholds* pThresh = pCmd->as<ExtColorThresholds>();
            if(pThresh)
            {
                ColorThreshold thresh = { pThresh->y[0], pThresh->y[1], pThresh->u[0], pThresh->u[1], pThresh->v[0], pThresh->v[1] };

                switch(pThresh->colorId)
                {
                    case EXT_COLOR_THRESHOLDS_ID_ORANGE:
                        classifier_.updateThreshold(Color::Orange, thresh);
                        break;
                    case EXT_COLOR_THRESHOLDS_ID_WHITE:
                        classifier_.updateThreshold(Color::White, thresh);
                        break;
                    case EXT_COLOR_THRESHOLDS_ID_GREEN:
                        classifier_.updateThreshold(Color::Green, thresh);
                        break;
                }
            }
        }
        break;
    }
}

void RegionsLocalisation::debugDrawBalls(const std::vector<RegionsLocalisation::DetectedBall>& balls, FrameYUV420* pFrame)
{
    std::stringstream str;
    for(const auto& ball : balls)
    {
        pFrame->drawCircle(ball.centroid.x(), ball.centroid.y(), 5, ColorRGB(0.0f, 1.0f, 0.0f));

        auto circle = ball.circleFit.getCircle();
        pFrame->drawCircle(circle.x() * 2, circle.y() * 2, circle.radius() * 2, ColorRGB(0.0f, 0.3f, 1.0f));

        str.str("");
        str << std::setprecision(4) << "R: " << ball.circleFit.getResidualNormalized();
        pFrame->drawText(ball.centroid.x(), ball.centroid.y() - 60, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);

        str.str("");
        str << std::setprecision(4) << "A: " << ball.posFromDiameter_base.x() << ", "
            << ball.posFromDiameter_base.y() << ", " << ball.posFromDiameter_base.z();
        pFrame->drawText(ball.centroid.x(), ball.centroid.y() - 40, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);

        str.str("");
        str << std::setprecision(4) << "B: " << ball.posFromPlaneIntersection_base.x() << ", "
            << ball.posFromPlaneIntersection_base.y() << ", " << ball.posFromPlaneIntersection_base.z();
        pFrame->drawText(ball.centroid.x(), ball.centroid.y() - 20, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);

        str.str("");
        str << std::setprecision(4) << "M: " << ball.posFromDiameter_map.x() << ", "
            << ball.posFromDiameter_map.y() << ", " << ball.posFromDiameter_map.z();
        pFrame->drawText(ball.centroid.x(), ball.centroid.y() - 0, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);
    }
}

void RegionsLocalisation::debugDrawTrackers(FrameYUV420* pFrame)
{
    std::stringstream str;

    for(const auto& tracker : trackers_)
    {
        Eigen::Vector2f imgPos = mapPosToDistortedImagePos(tracker.getPosition().cast<float>());

        str.str("");
        str << "ID: " << tracker.getId();
        pFrame->drawText(imgPos.x(), imgPos.y() + 60, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);

        str.str("");
        str << std::setprecision(4) << "TP: " << tracker.getPosition().x() << ", "
            << tracker.getPosition().y() << ", " << tracker.getPosition().z();
        pFrame->drawText(imgPos.x(), imgPos.y() + 20, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);

        str.str("");
        str << std::setprecision(4) << "TV: " << tracker.getVelocity().x() << ", "
            << tracker.getVelocity().y() << ", " << tracker.getVelocity().z();
        pFrame->drawText(imgPos.x(), imgPos.y() + 40, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);

        for(const auto& p : tracker.getHistory())
        {
            Eigen::Vector2f imgPos = mapPosToDistortedImagePos(p.pos.cast<float>());
            pFrame->drawCircle(imgPos.x(), imgPos.y(), 5, ColorRGB(0.0f, 1.0f, 1.0f));
        }

        LineFit lineFit = tracker.getLineFit();
        if(lineFit.isValid())
        {
            Eigen::Vector3f lineOrigin(0, 0, 0);
            lineOrigin.head<2>() = lineFit.getLine().support();

            Eigen::Vector3f lineDir(0, 0, 0);
            lineDir.head<2>() = lineFit.getLine().direction();

            Eigen::ParametrizedLine<float, 3> velRay_base(base_T_map_ * lineOrigin, base_T_map_.linear() * lineDir);
            Eigen::Hyperplane<float, 3> frontPlane_base(Eigen::Vector3f::UnitY(), Eigen::Vector3f(0, 0.10f, 0));

            const float lineLength = tracker.getVelocity().norm();
            float intersectParam = velRay_base.intersectionParameter(frontPlane_base);
            Eigen::Vector3f lineEnd_base;

            if(intersectParam < 0) // ball rolling away
            {
                lineEnd_base = velRay_base.pointAt(lineLength);
            }
            else if(intersectParam < lineLength) // intersect too far behind front plane
            {
                lineEnd_base = velRay_base.pointAt(intersectParam);
            }
            else
            {
                lineEnd_base = velRay_base.pointAt(lineLength);
            }

            Eigen::Vector3f startPos = lineOrigin;
            Eigen::Vector3f endPos = map_T_base_ * lineEnd_base;
            Eigen::Vector2f startImg = mapPosToDistortedImagePos(startPos);
            Eigen::Vector2f endImg = mapPosToDistortedImagePos(endPos);

            pFrame->drawLine(startImg.x(), startImg.y(), endImg.x(), endImg.y(), ColorRGB(0.0f, 1.0f, 0.5f));
        }
    }
}
