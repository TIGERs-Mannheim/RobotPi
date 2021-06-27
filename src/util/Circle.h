#pragma once

#include <vector>
#include <Eigen/Dense>

class Circle
{
public:
    Circle();
    Circle(float x, float y, float radius);

    float x() const { return x_; }
    float y() const { return y_; }
    float radius() const { return radius_; }

private:
    float x_;
    float y_;
    float radius_;
};

class CircleFit
{
public:
    CircleFit();
    CircleFit(const std::vector<Eigen::Vector2f>& points);

    Circle getCircle() const { return circle_; }
    bool isValid() const { return validFit_; }
    float getResidual() const { return residual_; }
    float getResidualNormalized() const { return residualNormalized_; }

private:
    Circle circle_;
    bool validFit_;
    float residual_;
    float residualNormalized_;
};
