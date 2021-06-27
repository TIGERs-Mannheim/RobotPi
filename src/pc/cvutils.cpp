/*
 * cvutils.cpp
 *
 *  Created on: 01.02.2021
 *      Author: FelixW
 */
#include "cvutils.h"

FrameYUV420 imageByPath(const char *path)
{
    cv::Mat image = cv::imread(path);
    
    FrameMetadata meta;
    meta.width = image.size[1];
    meta.height = image.size[0];
    
    cv::Mat yuvImg (meta.height, meta.width, CV_8UC3);
    cv::cvtColor(image, yuvImg, cv::COLOR_BGR2YUV);

    int size = image.size[0] * image.size[1];
    uint8_t* yData = new uint8_t[size];
    uint8_t* uData = new uint8_t[size/4];
    uint8_t* vData = new uint8_t[size/4];
    
    int halfWidth = meta.width/2;
    for(int y = 0; y < meta.height; y++)
    {
        for(int x = 0; x < meta.width; x++)
        {
            cv::Vec3b pixel = yuvImg.at<cv::Vec3b>(y,x);
            yData[y*meta.width+x] = pixel[0];
            uData[y/2*halfWidth+x/2] = pixel[1];
            vData[y/2*halfWidth+x/2] = pixel[2];
        }
    }
    
    return FrameYUV420(yData, uData, vData, meta);
}

cv::Mat frameToMat(FrameYUV420& yuvFrame)
{
    FrameMetadata meta = yuvFrame.getMetadata();
    cv::Mat_<cv::Vec3b> image (meta.height, meta.width, CV_8UC3);
    
    uint8_t* yData = yuvFrame.getYData();
    uint8_t* uData = yuvFrame.getUData();
    uint8_t* vData = yuvFrame.getVData();
    
    int halfWidth = meta.width/2;
    for(int y = 0; y < meta.height; y++)
    {
        for(int x = 0; x < meta.width; x++)
        {
            image(y, x)[0] = yData[y*meta.width+x];
            image(y, x)[1] = uData[y/2*halfWidth+x/2];
            image(y, x)[2] = vData[y/2*halfWidth+x/2];
        }
    }
    
    cv::cvtColor(image, image, cv::COLOR_YUV2BGR);
    return std::move ( image );
}

void showFrame(FrameYUV420& yuvFrame){
    cv::imshow("YUV Frame", frameToMat(yuvFrame));
    cv::waitKey(0);
    cv::destroyAllWindows();
}
