/*
 * TrackingFilterPosVel.h
 *
 *  Created on: 24.12.2020
 *      Author: AndreR
 */

#pragma once

#include "KalmanFilter.h"

template <typename _Scalar, int _Dims>
class TrackingFilterPosVel : public KalmanFilter<_Scalar, _Dims*2, _Dims, 1>
{
public:
    typedef Eigen::Matrix<_Scalar, _Dims, 1> PositionVector;
    typedef Eigen::Matrix<_Scalar, _Dims, 1> VelocityVector;
    typedef Eigen::Matrix<_Scalar, _Dims*2, 1> StateVector;

protected:
    using KalmanFilter<_Scalar, _Dims*2, _Dims, 1>::predict;

    _Scalar modelError;
    _Scalar lastTimestamp;

public:
    TrackingFilterPosVel()
    {
    }

    TrackingFilterPosVel(const PositionVector& initialPos, _Scalar covariance, _Scalar modelErr, _Scalar measErr, _Scalar timestamp)
    :modelError(modelErr), lastTimestamp(timestamp)
    {
        this->x.template head<_Dims>() = initialPos;
        this->P = KalmanFilter<_Scalar, _Dims*2, _Dims, 1>::CovarianceMatrix::Identity() * covariance;
        this->B.setZero();
        this->C.template block<_Dims, _Dims>(0, 0).setIdentity();

        setMeasurementError(measErr);
    }

    void setMeasurementError(_Scalar error)
    {
        this->Ez.diagonal().setConstant(error);
    }

    void setModelError(_Scalar error)
    {
        modelError = error;
    }

    PositionVector getPositionEstimate() const { return this->x.template head<_Dims>(); }
    VelocityVector getVelocityEstimate() const { return this->x.template tail<_Dims>(); }

    bool isNaN() const { return this->x.array().isNaN().any(); }

    void predict(_Scalar timestamp)
    {
        _Scalar dt = (timestamp - lastTimestamp);
        if(dt <= 0)
            return;

        lastTimestamp = timestamp;

        updateMatrices(dt);

        predict();
    }

private:
    void updateMatrices(_Scalar dt)
    {
        _Scalar sigma = sqrt((3*modelError)/dt)/dt;
        _Scalar dt3 = (1.0/3.0)*dt*dt*dt*sigma*sigma;
        _Scalar dt2 = (1.0/2.0)*dt*dt*sigma*sigma;
        _Scalar dt1 = dt*sigma*sigma;

        this->Ex.template block<_Dims, _Dims>(0, 0).diagonal().setConstant(dt3);
        this->Ex.template block<_Dims, _Dims>(0, _Dims).diagonal().setConstant(dt2);
        this->Ex.template block<_Dims, _Dims>(_Dims, 0).diagonal().setConstant(dt2);
        this->Ex.template block<_Dims, _Dims>(_Dims, _Dims).diagonal().setConstant(dt1);

        this->A.diagonal().setOnes();
        this->A.template block<_Dims, _Dims>(0, _Dims).diagonal().setConstant(dt);
    }
};
