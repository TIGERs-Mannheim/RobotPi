/*
 * RadialDistortion.h
 *
 *  Created on: 30.11.2020
 *      Author: AndreR
 */

#pragma once

#include <vector>
#include <Eigen/Dense>
#include "commands.h"

class Camera
{
public:
    Camera();

    void setCurrentResolutionWidth(uint32_t width);
    void setIntrinsics(ExtCameraIntrinsics intrinsics);

    Eigen::Vector3f getNormalizedRay(const Eigen::Vector2f& in) const;
    Eigen::Vector2f getImageCoordinates(const Eigen::Vector3f& in) const;

    Eigen::Vector2f undistort(const Eigen::Vector2f& in) const;
    Eigen::Vector2f distort(const Eigen::Vector2f& in) const;

    float getFocalLength() const { return focalLength_; }

    std::vector<Eigen::Vector2f> undistort(const std::vector<Eigen::Vector2f>& points) const;
    std::vector<Eigen::Vector2f> distort(const std::vector<Eigen::Vector2f>& points) const;

private:
    Eigen::Vector2f undistortCoeff_;
    Eigen::Vector2f distortCoeff_;
    Eigen::Vector2f principalPoint_;
    float focalLength_;

    ExtCameraIntrinsics intrinsics_;
};
