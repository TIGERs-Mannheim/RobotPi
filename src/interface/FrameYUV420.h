/*
 * FrameYUV420.h
 *
 *  Created on: 25.09.2020
 *      Author: AndreR
 */

#pragma once

#include "util/Color.h"
#include <string>

class FrameMetadata
{
public:
    uint32_t width;
    uint32_t height;
    int64_t timestampUs; // RPi STC (system time clock) at start of frame readout (not start of exposure!)
    uint32_t exposureUs;

    // timestamp for center of exposure is timestampUs - exposureUs/2
};

/**
 * Image data in YUV420 format.
 * Y = luma channel with full resolution
 * U,V = chroma channels with half resolution (height & width)
 */
class FrameYUV420
{
public:
    FrameYUV420(uint8_t* pYData, uint8_t* pUData, uint8_t* pVData, const FrameMetadata& metadata);

    uint8_t* getYData() const { return pYData_; }
    uint8_t* getUData() const { return pUData_; }
    uint8_t* getVData() const { return pVData_; }

    const FrameMetadata& getMetadata() const { return metadata_; }

    // draw point at (x,y) in full resolution coordinates
    void drawPoint(int32_t x, int32_t y, ColorYUV color);
    void drawPointThin(int32_t x, int32_t y, ColorYUV color);

    // draw line from (x1,y1) to (x2,y2) in full resolution coordinates
    void drawLine(float x1, float y1, float x2, float y2, ColorYUV color);

    // draw a circle at (x0,y0) with radius 'radius' in full resolution coordinates
    void drawCircle(float x0, float y0, float radius, ColorYUV color);

    // draw a rectangle from point (x0,y0) to (x1,y1) in full resolution coordinates
    void drawRect(float x0, float y0, float x1, float y1, ColorYUV color);

    // draw a rectangle from point (x0,y0) to (x1,y1) in full resolution coordinates
    void drawFilledRect(float x0, float y0, float x1, float y1, ColorYUV color);

    // draw text at point (x, y), lower left corner, in full resolution coordinates
    void drawText(int32_t x, int32_t y, std::string text, ColorYUV color, bool large = false);
private:
    uint8_t* pYData_;
    uint8_t* pUData_;
    uint8_t* pVData_;

    FrameMetadata metadata_;
};
