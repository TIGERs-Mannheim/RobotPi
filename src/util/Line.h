/*
 * Line.h
 *
 *  Created on: 26.12.2020
 *      Author: AndreR
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

class Line
{
public:
    Line();
    Line(const Eigen::Vector2f& support, const Eigen::Vector2f& direction);

    Eigen::Vector2f support() const { return support_; }
    Eigen::Vector2f direction() const { return direction_; }

private:
    Eigen::Vector2f support_;
    Eigen::Vector2f direction_;
};

class LineFit
{
public:
    LineFit();
    LineFit(const std::vector<Eigen::Vector2f>& pointList);

    Line getLine() const { return line_; }
    bool isValid() const { return validFit_; }
    float getResidual() const { return residual_; }

private:
    Line line_;
    bool validFit_;
    float residual_;
};
