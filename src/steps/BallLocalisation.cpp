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

   Pixel size: 1.4um x 1.4um
*/

#include <steps/BallLocalisation.h>
#include "util/log.h"
#include <iomanip>
#include "util/KalmanFilter.h"
#include "ColorClassifierYUV.h"
#include <iostream>

BallLocalisation::BallLocalisation()
:nextTrackerId_(0)
{
    config_.greedyCircleFactor = 1.25f;
    config_.maxTrackers = 5;
    config_.minBlobArea = 5;
    config_.topImageSkipFactor = 0.25f;
    config_.trackerTimeoutMs = 1000;
    config_.usePlaneIntersection = 0;

    addDefaultStepConfigHandler(EXT_STEP_MASK_BALL_LOC);

    addCommandHandler<ExtRobotState>(CMD_EXT_ROBOT_STATE, [&] (ExtRobotState* pState) {
        robotStates_.push_back(*pState);
        while(robotStates_.size() > 1000)
            robotStates_.pop_front();
    });
    addCommandHandler<ExtBallLocalisationConfig>(CMD_EXT_BALL_LOC_CONFIG, [&] (ExtBallLocalisationConfig* pConfig) {
        config_ = *pConfig;

        for(auto& tracker : trackers_)
        {
            tracker.setHistorySize(pConfig->historySize);
            tracker.setMaxVelocity(pConfig->maxVelocity);
            tracker.setMeasurementError(pConfig->measError);
            tracker.setModelError(pConfig->modelError);
        }
    });
}

void BallLocalisation::execute(State& state)
{
    // Camera model at chroma channel resolution
    state.camera.setCurrentResolutionWidth(state.uvWidth);
    auto& metadata = state.pFrame->getMetadata();

    // get robot timestamp at image time and update transforms
    const uint32_t ucTime = state.timeSync.stc2mc(metadata.timestampUs - metadata.exposureUs);
    ExtRobotState usedRobotState = updateTransforms(ucTime, state.pFrame);

    // classify orange regions (balls?)
    const auto& orangeList = state.frameColoredRegions[Color::Orange];

    std::vector<BallLocalisation::DetectedBall> balls;
    uint32_t topSkipRows = metadata.height * config_.topImageSkipFactor;

    for(const auto& reg : orangeList)
    {
        if(reg.y1 <= topSkipRows/2 || reg.area < config_.minBlobArea)
            continue;

        auto ball = processRegion(state, reg);
        if(ball.valid)
        {
            balls.push_back(ball);
        }
    }

    // sort by size of ball
    std::sort(balls.begin(), balls.end(), [](const BallLocalisation::DetectedBall& a, const BallLocalisation::DetectedBall& b) {
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

            if((center1 - center2).norm() < circle1.radius()*config_.greedyCircleFactor)
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

    const double tFrameStc = metadata.timestampUs * 1e-6;

    // remove trackers not updated for some time
    trackers_.erase(std::remove_if(trackers_.begin(), trackers_.end(),
        [=](const auto& t){
        return (tFrameStc - t.getLastCaptureTimestamp()) > config_.trackerTimeoutMs*1e-3f || t.isNaN(); }),
        trackers_.end());

    // do a prediction on all trackers
    for(auto& tracker : trackers_)
        tracker.predict(tFrameStc);

    // iterate over all balls
    for(size_t i = 0; i < balls.size() && i < config_.maxTrackers; i++)
    {
        const auto& ball = balls[i];
        bool consumed = false;
        Eigen::Vector3d usedPos = config_.usePlaneIntersection ? ball.posFromPlaneIntersection_map.cast<double>() : ball.posFromDiameter_map.cast<double>();

        // offer ball to all trackers
        for(auto& tracker : trackers_)
        {
            if(tracker.update(tFrameStc, usedPos))
            {
                consumed = true;
                break;
            }
        }

        // create new tracker if nobody wanted this ball
        if(!consumed && trackers_.size() < config_.maxTrackers)
        {
            BallTracker tracker(nextTrackerId_, usedPos, tFrameStc);
            tracker.setHistorySize(config_.historySize);
            tracker.setMaxVelocity(config_.maxVelocity);
            tracker.setMeasurementError(config_.measError);
            tracker.setModelError(config_.modelError);

            trackers_.push_back(tracker);
            nextTrackerId_++;
        }
    }

    std::sort(trackers_.begin(), trackers_.end(), [&](const BallTracker& a, const BallTracker& b) {
       return (base_T_map_ * a.getPosition().cast<float>()).norm() < (base_T_map_ * b.getPosition().cast<float>()).norm();
    });

    // draw debug information
    if(getDebugLevel())
    {
        if(config_.topImageSkipFactor > 0.0f)
        {
            state.pFrame->drawLine(0, topSkipRows, metadata.width, topSkipRows, ColorRGB(0.3f, 1.0f, 0.3f));
        }
    }

    if(getDebugLevel() > 1)
        debugDrawBalls(balls, state.pFrame);

    if(getDebugLevel() > 2)
        debugDrawTrackers(state.camera, state.pFrame);

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

    state.pComm->write({CMD_EXT_BALL_DETECTIONS, detections});
}

Eigen::Vector2f BallLocalisation::mapPosToDistortedImagePos(const Camera& camera, const Eigen::Vector3f& pos_map)
{
    Eigen::Vector3f pos_cam = camera.getBaseToCam() * base_T_map_ * pos_map;
    if(pos_cam.z() < 0.001f)
        return Eigen::Vector2f(NAN, NAN);

    return camera.distort(camera.getImageCoordinates(pos_cam))*2;
}

ExtRobotState BallLocalisation::updateTransforms(uint32_t ucTime, FrameYUV420* pFrame)
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
        map_T_base_ = Eigen::AngleAxisf(-M_PI/2.0f, Eigen::Vector3f::UnitZ());
        base_T_map_ = map_T_base_.inverse(Eigen::Affine);
        memset(&usedState, 0, sizeof(ExtRobotState));
    }

    if(getDebugLevel())
    {
        std::stringstream str;
        str << std::setprecision(4) << "map_pos: " << minIter->posGlobal[0] << ", "
            << minIter->posGlobal[1] << ", " << minIter->posGlobal[2];
        pFrame->drawText(5, 15, str.str(), ColorRGB(1.0f, 1.0f, 1.0f), false);
    }

    return usedState;
}

BallLocalisation::DetectedBall BallLocalisation::processRegion(State& state, const ColoredRegion& region)
{
    std::vector<Eigen::Vector2f> edges;

    // find edges of ball
    for(uint32_t row = region.y1; row < region.y2; row++)
    {
        const uint8_t* pRow = state.classifiedFrameData.data() + row * state.uvWidth;

        if(region.x1 > 0)
        {
            // region not at left image edge

            for(uint32_t col = region.x1; col < region.x2; col++)
            {
                if(pRow[col] == region.color)
                {
                    edges.push_back({col, row});
                    break;
                }
            }
        }

        if(region.x2 < (uint32_t)state.uvWidth-3)
        {
            // region not at right image edge:
            // note that -1 would work, but the RPi camera outputs a white strip on the right edge of the image
            // in 1280x960 mode, hence we use -3 to ignore this strip

            for(uint32_t col = region.x2; col > region.x1; col--)
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
    auto undistorted = state.camera.undistort(edges);

    // draw debug edges
    if(getDebugLevel() > 1)
    {
        for(const auto& edge : edges)
        {
            state.pFrame->drawPoint(edge.x()*2, edge.y()*2, ColorRGB(1.0f, 0.0f, 0.0f));
        }

        for(const auto& p : undistorted)
        {
            state.pFrame->drawPoint(p.x()*2, p.y()*2, ColorRGB(1.0f, 1.0f, 0.0f));
        }
    }

    // fit circle and determine 3D position
    BallLocalisation::DetectedBall detectedBall;
    detectedBall.valid = false;

    CircleFit fit(undistorted);
    if(fit.isValid() && fit.getResidualNormalized() < 1000)
    {
        Circle circle = fit.getCircle();

        detectedBall.valid = true;
        detectedBall.circleFit = fit;
        detectedBall.centroid = Eigen::Vector2i(region.cen_x*2, region.cen_y*2);
        detectedBall.area = region.area;

        const float focalLength = state.camera.getFocalLength();

        // determine 3D pos from circle diameter
        float distance = BALL_DIAMETER * focalLength / (circle.radius()*2.0f); // Z plane distance, NOT distance to ball
        Eigen::Vector3f cam_ball = state.camera.getNormalizedRay(Eigen::Vector2f(circle.x(), circle.y())) * distance;
        detectedBall.posFromDiameter_base = state.camera.getCamToBase() * cam_ball;
        detectedBall.posFromDiameter_map = map_T_base_ * detectedBall.posFromDiameter_base;

        // determine 3D pos from floor intersection (assuming ball is not airborne)
        Eigen::Vector3f cam_ray = state.camera.getNormalizedRay(Eigen::Vector2f(circle.x(), circle.y())).normalized();
        auto base_ray = state.camera.getCamToBase().linear() * cam_ray;
        Eigen::ParametrizedLine<float, 3> cam_rayLine(state.camera.getCamToBase().translation(), base_ray);
        Eigen::Hyperplane<float, 3> floor(Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitZ()*(BALL_DIAMETER/2));

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

void BallLocalisation::debugDrawBalls(const std::vector<BallLocalisation::DetectedBall>& balls, FrameYUV420* pFrame)
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

void BallLocalisation::debugDrawTrackers(const Camera& camera, FrameYUV420* pFrame)
{
    std::stringstream str;

    for(const auto& tracker : trackers_)
    {
        Eigen::Vector2f imgPos = mapPosToDistortedImagePos(camera, tracker.getPosition().cast<float>());

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
            Eigen::Vector2f imgPos = mapPosToDistortedImagePos(camera, p.pos.cast<float>());
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
            Eigen::Vector2f startImg = mapPosToDistortedImagePos(camera, startPos);
            Eigen::Vector2f endImg = mapPosToDistortedImagePos(camera, endPos);

            pFrame->drawLine(startImg.x(), startImg.y(), endImg.x(), endImg.y(), ColorRGB(0.0f, 1.0f, 0.5f));
        }
    }
}
