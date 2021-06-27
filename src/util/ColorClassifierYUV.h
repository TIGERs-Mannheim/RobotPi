/*
 * ColorClassifierYUV.h
 *
 *  Created on: 22.11.2020
 *      Author: AndreR
 *
 * Offers fast color classification in up to eight colors.
 * Based on: "Fast and Inexpensive Color Image Segmentation for Interactive Robots"
 * by Bruce, J. et al (2000)
 */

#pragma once

#include "interface/FrameYUV420.h"
#include "util/Color.h"

#include <cstdint>
#include <vector>
#include <map>

enum class Color
{
    White,
    Orange,
    Yellow,
    Green,
    Blue,
    Magenta
};

struct ColorThreshold
{
    uint8_t y[2];
    uint8_t u[2];
    uint8_t v[2];
};

class ColorClassifierYUV
{
public:
    struct Output
    {
        const uint8_t* pColorClass;
        uint32_t width;
        uint32_t height;
    };

    ColorClassifierYUV();

    Output process(FrameYUV420* pFrame);

    void updateThreshold(Color color, const ColorThreshold& threshold);

private:
    void processRows(FrameYUV420* pFrame, uint32_t startRow, uint32_t endRow);

    void updateLut();

    std::vector<uint8_t> result_;

    std::map<Color, ColorThreshold> thresholds_;
    std::vector<ColorYUV> colorMap_;

    bool multithreaded_;
    bool drawColors_;

    uint8_t lutY_[256];
    uint8_t lutU_[256];
    uint8_t lutV_[256];
};
