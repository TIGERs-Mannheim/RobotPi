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
#include "util/WorkerPool.h"
#include "interface/ProcessingStep.h"

#include <cstdint>
#include <vector>
#include <map>

struct ColorThreshold
{
    uint8_t y[2];
    uint8_t u[2];
    uint8_t v[2];
};

class ColorClassifierYUV : public AProcessingStep
{
public:

    ColorClassifierYUV();

    void execute(State& state) override;

    void updateThreshold(Color color, const ColorThreshold& threshold);

private:
    struct ClassificationJob
    {
        const uint8_t* pY;
        const uint8_t* pU;
        const uint8_t* pV;
        uint32_t uvWidth;
        uint8_t* pOut;
        uint32_t startRow;
        uint32_t endRow;
        FrameYUV420* pFrame;
    };

    void processRows(ClassificationJob jobData);
    void onColorThresholds(ExtColorThresholds* pThresh);

    void updateLut();

    std::map<Color, ColorThreshold> thresholds_;
    std::vector<ColorYUV> colorMap_;

    uint8_t lutY_[256];
    uint8_t lutU_[256];
    uint8_t lutV_[256];
};
