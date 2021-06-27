/*
 * RadialDistortion.cpp
 *
 *  Created on: 30.11.2020
 *      Author: AndreR
 */

#include <util/Camera.h>

Camera::Camera()
:focalLength_(0)
{
    intrinsics_.resolution[0] = 2592;
    intrinsics_.resolution[1] = 1944;
    intrinsics_.focalLength = 1342.0f;
    intrinsics_.principalPoint[0] = 1296.0f;
    intrinsics_.principalPoint[1] = 972.0f;
    intrinsics_.radialDistortion[0] = 0;
    intrinsics_.radialDistortion[1] = 0;
    setIntrinsics(intrinsics_); //Calc (un)distortion
}

void Camera::setCurrentResolutionWidth(uint32_t width)
{
    float factor = (float)width/intrinsics_.resolution[0];

    principalPoint_ = Eigen::Vector2f(intrinsics_.principalPoint[0], intrinsics_.principalPoint[1]) * factor;
    focalLength_ = intrinsics_.focalLength * factor;
}

void Camera::setIntrinsics(ExtCameraIntrinsics intrinsics)
{
    distortCoeff_ = { intrinsics.radialDistortion[0], intrinsics.radialDistortion[1] };

    // taken from: "An Exact Formula for Calculating Inverse Radial Lens Distortions" Drap, P. et al (2016)
    const auto& k = distortCoeff_;
    undistortCoeff_ = { -k(0), 3*k(0)*k(0)-k(1) };

    intrinsics_ = intrinsics;
}

Eigen::Vector3f Camera::getNormalizedRay(const Eigen::Vector2f& in) const
{
    Eigen::Vector3f result;
    result.head(2) = (in - principalPoint_) * 1.0f/focalLength_;
    result(2) = 1.0f;

    return result;
}

Eigen::Vector2f Camera::getImageCoordinates(const Eigen::Vector3f& in) const
{
    return in.head(2)/in.z()*focalLength_ + principalPoint_;
}

Eigen::Vector2f Camera::undistort(const Eigen::Vector2f& in) const
{
    Eigen::Vector2f n = (in - principalPoint_) * 1.0f/focalLength_;

    const float r2 = n.x()*n.x() + n.y()*n.y();
    const float r4 = r2*r2;
    const float f = 1.0f + undistortCoeff_(0)*r2 + undistortCoeff_(1)*r4;

    return n*f*focalLength_ + principalPoint_;
}

Eigen::Vector2f Camera::distort(const Eigen::Vector2f& in) const
{
    Eigen::Vector2f n = (in - principalPoint_) * 1.0f/focalLength_;

    const float r2 = n.x()*n.x() + n.y()*n.y();
    const float r4 = r2*r2;
    const float f = 1.0f + distortCoeff_(0)*r2 + distortCoeff_(1)*r4;

    return n*f*focalLength_ + principalPoint_;
}

std::vector<Eigen::Vector2f> Camera::undistort(const std::vector<Eigen::Vector2f>& points) const
{
    std::vector<Eigen::Vector2f> out;

    for(const auto& p : points)
        out.push_back(undistort(p));

    return out;
}

std::vector<Eigen::Vector2f> Camera::distort(const std::vector<Eigen::Vector2f>& points) const
{
    std::vector<Eigen::Vector2f> out;

    for(const auto& p : points)
        out.push_back(distort(p));

    return out;
}
