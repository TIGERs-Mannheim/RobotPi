/*
 * BallTracker.h
 *
 *  Created on: 25.12.2020
 *      Author: AndreR
 */

#pragma once

#include "TrackingFilterPosVel.h"
#include "util/Line.h"
#include <deque>
#include <cstdint>

class BallTracker
{
public:
    struct HistoryEntry
    {
        double timestamp;
        Eigen::Vector3d pos;
    };

    BallTracker(uint32_t id, const Eigen::Vector3d& initialPos, double tNow);

    void predict(double tNow);
    bool update(double tNow, const Eigen::Vector3d& position);

    void setModelError(double error) { modelError_ = error; }
    void setMeasurementError(double error) { measError_ = error; }
    void setMaxVelocity(double vel) { maxVelocity_ = vel; }
    void setHistorySize(uint32_t size) { historySize_ = size; }

    Eigen::Vector3d getPosition() const { return filter_.getPositionEstimate(); }
    Eigen::Vector3d getVelocity() const { return filter_.getVelocityEstimate(); }
    double getLastCaptureTimestamp() const { return tLastCapture_; }
    LineFit getLineFit() const { return lineFit_; }
    uint32_t getId() const { return id_; }

    bool isNaN() const { return filter_.isNaN(); }

    const std::deque<HistoryEntry>& getHistory() const { return history_; }

private:
    uint32_t id_;

    double modelError_;
    double measError_;
    double maxVelocity_;
    uint32_t historySize_;

    TrackingFilterPosVel<double, 3> filter_;
    double tLastCapture_;

    LineFit lineFit_;

    std::deque<HistoryEntry> history_;
};
