/*
 * RadialDistortion.cpp
 *
 *  Created on: 30.11.2020
 *      Author: AndreR
 */

#include <util/Camera.h>
#include "util/nlohmann/json.h"

#include <utility>
#include <filesystem>
#include <fstream>

void to_json(nlohmann::json& j, const ExtCameraCalibration& params)
{
    j = nlohmann::json
    {
            {"resolutionX", params.resolutionX},
            {"resolutionY", params.resolutionY},
            {"focalLength", params.focalLength},
            {"principalPointX", params.principalPointX},
            {"principalPointY", params.principalPointY},
            {"distortionCoeff0", params.distortionCoeff0},
            {"distortionCoeff1", params.distortionCoeff1},
            {"height", params.height},
            {"rotationY", params.rotationY},
            {"rotationP", params.rotationP},
            {"rotationR", params.rotationR},
            {"score", params.score}
    };
}

void from_json(const nlohmann::json& j, ExtCameraCalibration& params)
{
    j.at("resolutionX").get_to(params.resolutionX);
    j.at("resolutionY").get_to(params.resolutionY);
    j.at("focalLength").get_to(params.focalLength);
    j.at("principalPointX").get_to(params.principalPointX);
    j.at("principalPointY").get_to(params.principalPointY);
    j.at("distortionCoeff0").get_to(params.distortionCoeff0);
    j.at("distortionCoeff1").get_to(params.distortionCoeff1);
    j.at("height").get_to(params.height);
    j.at("rotationY").get_to(params.rotationY);
    j.at("rotationP").get_to(params.rotationP);
    j.at("rotationR").get_to(params.rotationR);
    j.at("score").get_to(params.score);
}


Camera::Camera()
{
    applyCalibration();
}


Camera::Camera(const std::string& path): Camera()
{
    if(!std::filesystem::exists(path))
        return;

    std::ifstream file(path, std::ios::binary | std::ios::in);
    nlohmann::json jsonParams = nlohmann::json::parse(file);
    calibration_ = jsonParams.template get<ExtCameraCalibration>();

    strcpy(calibration_.status, "Calibration from file");
    applyCalibration();
}

void Camera::saveCalibration(const std::string& path)
{
    nlohmann::json jsonParams = calibration_;

    std::ofstream file(path, std::ios::binary | std::ios::out);
    file << jsonParams.dump(4);
}

void Camera::setCurrentResolutionWidth(uint32_t width)
{
    float factor = (float)width/(float)calibration_.resolutionX;

    principalPoint_ = Eigen::Vector2f(calibration_.principalPointX, calibration_.principalPointY) * factor;
    resolution_ = Eigen::Vector2i((float)calibration_.resolutionX * factor, (float)calibration_.resolutionY * factor);
    focalLength_ = calibration_.focalLength * factor;
}

void Camera::applyCalibration()
{
    distortCoeff_ = Eigen::Vector2f(calibration_.distortionCoeff0, calibration_.distortionCoeff1);
    // taken from: "An Exact Formula for Calculating Inverse Radial Lens Distortions" Drap, P. et al (2016)
    const auto& k = distortCoeff_;
    undistortCoeff_ = { -k(0), 3*k(0)*k(0)-k(1) };

    camToBaseOffset_(2) = calibration_.height;
    cameraRotationMatrix_ = Eigen::AngleAxisf(calibration_.rotationY, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(calibration_.rotationP, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(calibration_.rotationR, Eigen::Vector3f::UnitZ());
    camToBase_ = Eigen::Translation3f(camToBaseOffset_) * cameraRotationMatrix_;
    baseToCam_ = camToBase_.inverse(Eigen::Affine);

    setCurrentResolutionWidth(resolution_(0));
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

Eigen::Affine3f Camera::getCamToBase() const
{
    return camToBase_;
}

Eigen::Affine3f Camera::getBaseToCam() const
{
    return baseToCam_;
}

Eigen::Vector2f Camera::intersectRayWithGround(const Eigen::Vector3f& ray, float offset) const
{
    Eigen::Vector3f px_vector = cameraRotationMatrix_ * ray;
    float m = -(camToBaseOffset_(2)-offset) / px_vector(2);
    if(m < 0)
        return {NAN, NAN};
    else
        return {m * px_vector(0) + camToBaseOffset_(0), m * px_vector(1) + camToBaseOffset_(1)};
}

Eigen::Vector3f Camera::camToBase(const Eigen::Vector3f& cam) const
{
    return camToBaseOffset_ + cameraRotationMatrix_ * cam;
}
