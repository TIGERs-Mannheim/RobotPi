/*
 * Line.cpp
 *
 *  Created on: 26.12.2020
 *      Author: AndreR
 */

#include "Line.h"

Line::Line()
:support_(0, 0), direction_(0, 0)
{
}

Line::Line(const Eigen::Vector2f& support, const Eigen::Vector2f& direction)
:support_(support), direction_(direction)
{
}

LineFit::LineFit()
:validFit_(false), residual_(0)
{
}

LineFit::LineFit(const std::vector<Eigen::Vector2f>& pointList)
{
    validFit_ = false;
    residual_ = 0;

    if(pointList.size() < 2)
        return;

    Eigen::MatrixXf points(pointList.size(), 2);
    for(size_t i = 0; i < pointList.size(); i++)
    {
        points.row(i) = pointList[i].transpose();
    }

    Eigen::RowVector2f mean = points.colwise().mean();

    points.rowwise() -= mean;

    auto svd = points.jacobiSvd(Eigen::ComputeThinV);
    Eigen::Vector2f direction = svd.matrixV().col(0).normalized();
    residual_ = svd.singularValues()(1);

    Eigen::Vector2f oldToNewVec = pointList.back() - pointList.front();

    // make sure direction points towards travel line
    if(oldToNewVec.normalized().dot(direction) > 0)
        direction *= -1;

    line_ = Line(mean.transpose(), direction);
    validFit_ = true;
}
