/*
 * BallTracker.cpp
 *
 *  Created on: 25.12.2020
 *      Author: AndreR
 */

#include "BallTracker.h"

BallTracker::BallTracker(uint32_t id, const Eigen::Vector3d& initialPos, double tNow)
:id_(id),
 modelError_(0.1),
 measError_(10.0),
 maxVelocity_(10.0),
 historySize_(10),
 filter_(initialPos, 1.0, modelError_, measError_, tNow),
 tLastCapture_(tNow)
{
}

void BallTracker::predict(double tNow)
{
    filter_.predict(tNow);
}

bool BallTracker::update(double tNow, const Eigen::Vector3d& position)
{
    double dt = tNow - tLastCapture_;
    if(dt <= 0)
        return false;

    double distanceToPrediction = (filter_.getPositionEstimate() - position).norm();
    if(distanceToPrediction > dt*maxVelocity_)
        return false;

    filter_.setMeasurementError(measError_);
    filter_.setModelError(modelError_);

    filter_.correct(position);
    tLastCapture_ = tNow;

    HistoryEntry entry = { tNow, position };
    history_.push_front(entry);
    while(history_.size() > historySize_)
        history_.pop_back();

    std::vector<Eigen::Vector2f> historyVec;
    for(const auto& p : history_)
        historyVec.push_back(p.pos.head(2).cast<float>());

    lineFit_ = LineFit(historyVec);

    return true;
}

