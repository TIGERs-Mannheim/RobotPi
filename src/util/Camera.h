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

#define CAMERA_CALIBRATION_FILE_PATH "/opt/calibration.json"

class Camera
{
public:
    Camera();
    explicit Camera(const std::string& path);

    void saveCalibration(const std::string& path);
    ExtCameraCalibration& getCalibration() { return calibration_; }

    void setCurrentResolutionWidth(uint32_t width);
    void applyCalibration();

    Eigen::Vector3f getNormalizedRay(const Eigen::Vector2f& in) const;
    Eigen::Vector2f getImageCoordinates(const Eigen::Vector3f& in) const;

    Eigen::Vector2f intersectRayWithGround(const Eigen::Vector3f& ray, float offset) const;
    Eigen::Vector3f camToBase(const Eigen::Vector3f& cam) const;

    Eigen::Affine3f getCamToBase() const;
    Eigen::Affine3f getBaseToCam() const;

    Eigen::Vector2f undistort(const Eigen::Vector2f& in) const;
    Eigen::Vector2f distort(const Eigen::Vector2f& in) const;

    float getFocalLength() const { return focalLength_; }
    const Eigen::Vector2f& getPrincipalPoint() const { return principalPoint_; }
    const Eigen::Vector2i& getResolution() const { return resolution_; }

    std::vector<Eigen::Vector2f> undistort(const std::vector<Eigen::Vector2f>& points) const;
    std::vector<Eigen::Vector2f> distort(const std::vector<Eigen::Vector2f>& points) const;
private:
    Eigen::Vector2f undistortCoeff_;
    Eigen::Vector2f distortCoeff_;
    Eigen::Vector2f principalPoint_;
    Eigen::Vector2i resolution_;
    float focalLength_ = 0;

    ExtCameraCalibration calibration_ = {"Uncalibrated", 2592, 1944, 1330.0f, 1320.0f, 980.0f, -0.323f, 0.0407f, 0.08f, -1.91986f, 0.0f, 0.0f, 0.0f};

    Eigen::Affine3f camToBase_;
    Eigen::Affine3f baseToCam_;

    Eigen::Vector3f camToBaseOffset_ = Eigen::Vector3f(0.0f, 0.06f, 0.08f);
    Eigen::Matrix3f cameraRotationMatrix_;
};