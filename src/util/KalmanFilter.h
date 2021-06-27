/*
 * KalmanFilter.h
 *
 *  Created on: 24.12.2020
 *      Author: AndreR
 */

#pragma once

#include <Eigen/Dense>

template <typename _Scalar, int _States, int _Measurements, int _Controls>
class KalmanFilter
{
public:
    typedef Eigen::Matrix<_Scalar, _States, 1> StateVector;
    typedef Eigen::Matrix<_Scalar, _States, _States> CovarianceMatrix;
    typedef Eigen::Matrix<_Scalar, _Controls, 1> ControlVector;
    typedef Eigen::Matrix<_Scalar, _Measurements, 1> MeasurementVector;

protected:
    StateVector x; // state estimate
    CovarianceMatrix P; // state uncertainty

    Eigen::Matrix<_Scalar, _States, _States> A; // state transition
    Eigen::Matrix<_Scalar, _States, _Controls> B; // control matrix
    Eigen::Matrix<_Scalar, _Measurements, _States> C; // measurement matrix

    Eigen::Matrix<_Scalar, _States, _States> Ex; // process noise covariance
    Eigen::Matrix<_Scalar, _Measurements, _Measurements> Ez; // measurement noise covariance
    Eigen::Matrix<_Scalar, _Measurements, 1> N; // innovation

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    const StateVector& getStateEstimation() const { return x; }
    const CovarianceMatrix& getCovarianceMatrix() const { return P; }
    const MeasurementVector& getInnovation() const { return N; }

    KalmanFilter()
    {
        x.setZero();
        P.setZero();
        A.setZero();
        B.setZero();
        C.setZero();
        Ex.setZero();
        Ez.setZero();
        N.setZero();
    }

    void predict()
    {
        x = A*x;
        P = A*P*A.transpose() + Ex;
    }

    void predict(const ControlVector& u)
    {
        x = A*x + B*u;
        P = A*P*A.transpose() + Ex;
    }

    void correct(const MeasurementVector& z)
    {
        N = z - C*x;
        Eigen::Matrix<_Scalar, _Measurements, _Measurements> S = C*P*C.transpose() + Ez;
        Eigen::Matrix<_Scalar, _States, _Measurements> K = S.transpose().colPivHouseholderQr().solve(C*P.transpose()).transpose();
        x = x + K*N;
        P = (Eigen::Matrix<_Scalar, _States, _States>::Identity() - K*C) * P;
    }
};
