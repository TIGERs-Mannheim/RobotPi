#include "Circle.h"

Circle::Circle()
:x_(0), y_(0), radius_(0)
{
}

Circle::Circle(float x, float y, float radius)
:x_(x), y_(y), radius_(radius)
{
}

CircleFit::CircleFit()
:validFit_(false), residual_(0), residualNormalized_(0)
{
}

CircleFit::CircleFit(const std::vector<Eigen::Vector2f>& edges)
:validFit_(false), residual_(0), residualNormalized_(0)
{
    // Determine circle from points
    //    (px+cx)^2 + (py+cy)^2 = r^2
    // => cx^2 + cy^2 - r^2 - 2*px*cx - 2*py*cy = -(px^2 + py^2)
    // set: P = cx^2 + cy^2 - r^2, Q = -2*cx, R = -2*cy
    // => P + Q*px + R*py = -(px^2 + py^2)
    // vector x = [P Q R]'
    // matrix A = [1 px py; ..]
    // vector b = [-(px^2 + py^2); ..]

    if(edges.size() < 3)
        return;

    Eigen::MatrixXf A(edges.size(), 3);
    Eigen::VectorXf b(edges.size());

    for(size_t i = 0; i < edges.size(); i++)
    {
        const auto& p = edges[i];

        A(i, 0) = 1.0f;
        A(i, 1) = p.x();
        A(i, 2) = p.y();

        b(i) = -(p.x()*p.x() + p.y()*p.y());
    }

    Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

    Eigen::VectorXf residuals = b - A*x;
    residual_ = residuals.norm();
    residualNormalized_ = residual_/edges.size();
    validFit_ = true;

    float cx = x(1)/-2.0f;
    float cy = x(2)/-2.0f;
    float r = sqrtf(cx*cx + cy*cy - x(0));

    circle_ = Circle(cx, cy, r);
}
