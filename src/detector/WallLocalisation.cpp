/*
 * WallLocalisation.cpp
 *
 *  Created on: 22.05.2021
 *      Author: FelixW
 */

#include "WallLocalisation.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <bits/stdc++.h>

class Wall
{
public:
    Wall();
    
    void add(std::vector<Eigen::Vector3f>::iterator& begin, std::vector<Eigen::Vector3f>::iterator& end);
    void add(Wall& wall);
    double getAngle() const;
    double getDistance() const;
    unsigned int getWeight() const;
    Eigen::Vector3f& front();
    Eigen::Vector3f& back();
    
private:
    std::vector<Eigen::Vector3f> points_;
    double angle_;
    double distance_;
};

Wall::Wall()
:points_(std::vector<Eigen::Vector3f>()), angle_ (0.0), distance_ (0.0)
{
}

void Wall::add(std::vector<Eigen::Vector3f>::iterator& begin, std::vector<Eigen::Vector3f>::iterator& end)
{
    points_.insert(points_.end(), begin, end);
    
    const int size = points_.size();
    const int halfSize = size / 2;
    double x1 = 0;
    double y1 = 0;
    for(int i = 0; i < halfSize; i++)
    {
        x1 += points_[i].x();
        y1 += points_[i].y();
    }
    
    double x2 = 0;
    double y2 = 0;
    for(int i = halfSize; i < size; i++)
    {
        x2 += points_[i].x();
        y2 += points_[i].y();
    }
    
    // Compute the center point of the Wall
    Eigen::Vector2f center = Eigen::Vector2f((x1 + x2) / size, (y1 + y2) / size);
    
    // Compute the center point of the first (1) and second (2) half of the wall
    x1 /= halfSize;
    y1 /= halfSize;
    x2 /= size - halfSize;
    y2 /= size - halfSize;
    
    // Angle to bot
    angle_ = atan2(y2 - y1, x2 - x1);
    // Distance bot - wall at a 90deg angle to the wall
    distance_ = sin( angle_ ) * (center.y() / tan( angle_ ) - center.x());
}

void Wall::add(Wall& wall)
{
    std::vector<Eigen::Vector3f>::iterator begin = wall.points_.begin();
    std::vector<Eigen::Vector3f>::iterator end = wall.points_.end();
    add(begin, end);
}

double Wall::getAngle() const
{
    return angle_; //-pi bis pi
}

double Wall::getDistance() const
{
    return distance_;
}

unsigned int Wall::getWeight() const
{
    return points_.size();
}

Eigen::Vector3f& Wall::front()
{
    return points_.front();
}

Eigen::Vector3f& Wall::back()
{
    return points_.back();
}


std::ostream &operator<<(std::ostream &os, Wall const &wall)
{
    return os << "[" << wall.getDistance() << "m " << wall.getAngle() * 180 / EIGEN_PI << "° " << wall.getWeight() << "]";
}


WallLocalisation::WallLocalisation(std::shared_ptr<TimeSync> pTimeSync):
 debugLevel_(1),
 groundClearance_(0.0f),
 fieldRotation_(0.0f),
 fieldSizeX_(6.3f),
 fieldSizeY_(4.8f),
 pTimeSync_(pTimeSync)
{
    Eigen::Vector3f base_t_cam_ = Eigen::Vector3f(0, 0, 0.07);
    float camRotation = (-20.0f-90.0f)*M_PI/180.0f; // In z axis
    base_T_cam_ = Eigen::Translation3f(base_t_cam_) * Eigen::AngleAxisf(camRotation, Eigen::Vector3f::UnitX());
}

void WallLocalisation::setFieldRotation (float fieldRotation)
{
     fieldRotation_ = remainder(fieldRotation, 2*EIGEN_PI);
}

void WallLocalisation::processCommand(std::shared_ptr<Command> pCmd)
{
    switch(pCmd->getId())
    {
        case CMD_EXT_CAMERA_INTRINSICS:
        {
            const ExtCameraIntrinsics* pIntrinsics = pCmd->as<ExtCameraIntrinsics>();
            if(pIntrinsics)
            {
                camera_.setIntrinsics(*pIntrinsics);
            }
        }
        break;
        case CMD_EXT_CAMERA_EXTRINSICS:
        {
            const ExtCameraExtrinsics* pExtrinsics = pCmd->as<ExtCameraExtrinsics>();
            if(pExtrinsics)
            {
                Eigen::Vector3f base_t_cam_ = Eigen::Vector3f(pExtrinsics->cameraPos[0], pExtrinsics->cameraPos[1], pExtrinsics->cameraPos[2]);
                float camRotation_ = (-pExtrinsics->cameraTiltDeg - 90) * M_PI/180.0f;
                groundClearance_ = pExtrinsics->groundClearance;
                base_T_cam_ = Eigen::Translation3f(base_t_cam_) * Eigen::AngleAxisf(camRotation_, Eigen::Vector3f::UnitX());
            }
        }
        break;
        case CMD_EXT_FIELD_INFO:
        {
            const ExtFieldInfo* pState = pCmd->as<ExtFieldInfo>();
            if(pState)
            {
                fieldInfo_.push_back(*pState);

                while(fieldInfo_.size() > 1000)
                    fieldInfo_.pop_front();
            }
        }
        break;
    }
}

static void drawWall(FrameYUV420* pFrame, Camera const &camera, Eigen::Affine3f cam_T_base, Wall &wall, ColorYUV const &color)
{
    if(wall.getWeight() == 0)
        return;
    
    Eigen::Vector2f front = camera.distort(camera.getImageCoordinates(cam_T_base * wall.front()));
    Eigen::Vector2f back = camera.distort(camera.getImageCoordinates(cam_T_base * wall.back()));
    pFrame->drawLine(front.x(), front.y(), back.x(), back.y(), color);
}

Command::List WallLocalisation::processFrame(FrameYUV420* pFrame)
{
    const uint8_t blackDecider = 40;
    const uint32_t wallStartSize = 48;
    const double errorMargin = EIGEN_PI / 24; // 32 5,625deg
    
    const uint32_t width = pFrame->getMetadata().width;
    const uint32_t height = pFrame->getMetadata().height;
    
    // Find lowest Wall-colored pixel per column
    const uint8_t* yData = pFrame->getYData();
    int32_t pos[width];
    std::memset(pos, -1, width * sizeof(int));
    for(int32_t y = height-1; y >= 0; y--)
    {
        for(uint32_t x = 0; x < width; x++)
        {
            if(pos[x] == -1 && yData[y*width + x] < blackDecider)
                pos[x] = y;
        }
    }
    
    // Transform points from camera space to ground
    camera_.setCurrentResolutionWidth(width);
    Eigen::Hyperplane<float, 3> floor(Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitZ()*(-groundClearance_));
    std::vector<Eigen::Vector3f> points;
    for(uint32_t x = 0; x < width; x++)
    {
        Eigen::Vector3f cam_ray = camera_.getNormalizedRay(camera_.undistort(Eigen::Vector2f(x, pos[x]))).normalized();
        Eigen::ParametrizedLine<float, 3> cam_rayLine(base_T_cam_.translation(), base_T_cam_.linear() * cam_ray);

        float intersectionParam = cam_rayLine.intersectionParameter(floor);

        if(intersectionParam > 0)
        {
            cam_ray = cam_rayLine.intersectionPoint(floor);
            points.push_back(cam_ray);
        }

        if(debugLevel_)
            pFrame->drawPoint(x, pos[x], ColorYUV(255, 127, 127));
    }
    
    // Create the first walls from wallStartSize points
    std::vector<Wall> walls;
    std::vector<Eigen::Vector3f>::iterator begin = points.begin();
    for(uint32_t i = wallStartSize; i <= points.size(); i+=wallStartSize)
    {
        std::vector<Eigen::Vector3f>::iterator end = begin + wallStartSize;
        Wall wall;
        wall.add(begin, end);
        walls.push_back(wall);
        begin = end;
    }
    
    if(begin != points.end() && begin+1 != points.end())  //Guarantee at least 2 points for the last wall
    {
        Wall wall;
        std::vector<Eigen::Vector3f>::iterator end = points.end();
        wall.add(begin, end);
        walls.push_back(wall);
    }
    
    // Merge similar walls
    std::vector<Wall> mergedWalls;
    if(walls.size() > 0)
    {
        mergedWalls.push_back(walls[0]);
        walls.erase(walls.begin());

        for(Wall wall : walls)
        {
            if(abs(remainder(mergedWalls.back().getAngle() - wall.getAngle(), 2*EIGEN_PI)) < EIGEN_PI/2)
                mergedWalls.back().add(wall);
            else
                mergedWalls.push_back(wall);
        }
    }

    // Find nearest field info
    const uint32_t ucTime = pTimeSync_->stc2mc(pFrame->getMetadata().timestampUs - pFrame->getMetadata().exposureUs);
    const auto& minIter = std::min_element(fieldInfo_.begin(), fieldInfo_.end(), [&](const ExtFieldInfo& a, const ExtFieldInfo& b) {
        return std::abs((int32_t)(a.timestampUs - ucTime)) < std::abs((int32_t)(b.timestampUs - ucTime));
    });
    if(minIter != fieldInfo_.end())
    {
        fieldRotation_ = minIter->botOrientation;
        fieldSizeX_ = minIter->fieldSize[0]/2;
        fieldSizeY_ = minIter->fieldSize[1]/2;
    }
    
    // Generate position information
    Wall alignedWall[4];
    for(Wall wall : mergedWalls)
    {
        for(uint32_t i = 0; i < 4; i++)
        {
            if(abs(remainder(wall.getAngle() - fieldRotation_ - i * EIGEN_PI/2, 2*EIGEN_PI)) < errorMargin)
            {
                alignedWall[i].add(wall);
                break;
            }
        }
    }
    
    double position[2];
    for(uint32_t i = 0; i < 2; i++)
    {
        // Weighted average over both walls
        position[i] = ((fieldSizeX_ - alignedWall[i].getDistance()) * alignedWall[i].getWeight()
                       + (alignedWall[i+2].getDistance() - fieldSizeX_) * alignedWall[i+2].getWeight())
                    / ( alignedWall[i].getWeight() + alignedWall[i+2].getWeight());

        if(alignedWall[i].getWeight() == 0 && alignedWall[i+2].getWeight() == 0)
        {
            position[i] = NAN;
        }
    }

    // Draw recognized walls on the frame, print debug info
    if(debugLevel_)
    {
        Eigen::Affine3f cam_T_base = base_T_cam_.inverse(Eigen::Affine);

        std::cout << "Walls (" << mergedWalls.size() << "): ";
        for(Wall wall : mergedWalls)
        {
            drawWall(pFrame, camera_, cam_T_base, wall, ColorYUV(127, 255, 255));
            std::cout << wall << " ";
        }
        std::cout << std::endl;

        for(Wall wall : alignedWall)
        {
            drawWall(pFrame, camera_, cam_T_base, wall, ColorYUV(127, 0, 255));
        }
        
        std::string pos = "x: " +  std::to_string(position[0]) + " y: " + std::to_string(position[1]);
        std::cout << pos << std::endl;
        pFrame->drawText(width/2, height/2, pos, ColorYUV(255, 127, 127));
    }

    // Construct commands
    Command::List commands;

    ExtPositionDetection positionCommand;
    positionCommand.timestampUs = ucTime;
    positionCommand.pos[0] = position[0];
    positionCommand.pos[1] = position[1];
    commands.push_back(Command(CMD_EXT_POSITION_DETECTION, positionCommand));

    return commands;
}
