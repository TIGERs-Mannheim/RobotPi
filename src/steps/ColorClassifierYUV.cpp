/*
 * ColorClassifierYUV.cpp
 *
 *  Created on: 22.11.2020
 *      Author: AndreR
 */

#include "ColorClassifierYUV.h"
#include <cstring>
#include "commands.h"

ColorClassifierYUV::ColorClassifierYUV()
: colorMap_(256)
{
    updateThreshold(Color::Orange, { {0, 255}, {0, 128}, {144, 255} });
    updateThreshold(Color::White, {{144, 255}, {128 - 48, 128 + 64}, {128 - 64, 128 + 48} });
    updateThreshold(Color::Black, {{0, 32}, {128 - 16, 128 + 16}, {128 - 16, 128 + 32} });

    addDefaultStepConfigHandler(EXT_STEP_MASK_COLOR_CLASSIFIER);

    addCommandHandler<ExtColorThresholds>(CMD_EXT_COLOR_THRESHOLDS, std::bind(&ColorClassifierYUV::onColorThresholds, this, std::placeholders::_1));
}

void ColorClassifierYUV::execute(State& state)
{
    const uint32_t numPixelsUV = state.uvWidth * state.uvHeight;

    if(state.classifiedFrameData.size() < numPixelsUV)
        state.classifiedFrameData.resize(numPixelsUV);

    const unsigned int threads = state.threadPool.getNumThreads();
    std::vector<std::future<void>> jobs(threads);

    for(uint32_t i = 0; i < threads; i++)
    {
        ClassificationJob job;
        job.pY = state.pFrameYDataUVSized;
        job.pU = state.pFrame->getUData();
        job.pV = state.pFrame->getVData();
        job.uvWidth = state.uvWidth;
        job.pOut = state.classifiedFrameData.data();
        job.startRow = state.uvHeight/threads*i;
        job.endRow = state.uvHeight/threads*(i+1);
        job.pFrame = state.pFrame;

        jobs[i] = state.threadPool.run<void>(std::bind(&ColorClassifierYUV::processRows, this, job));
    }

    state.threadPool.waitFor(jobs);
}

#define LUT_FASTER_THRESHOLD 4
void ColorClassifierYUV::processRows(ClassificationJob jobData)
{
    uint32_t uvWidth = jobData.uvWidth;

    // row and col in uv coordinates
    for(uint32_t row = jobData.startRow; row < jobData.endRow; row++)
    {
        const uint8_t* pYRow = jobData.pY + row * uvWidth;
        const uint8_t* pURow = jobData.pU + row * uvWidth;
        const uint8_t* pVRow = jobData.pV + row * uvWidth;

        uint8_t* pOutRow = jobData.pOut + row * uvWidth;

        if(thresholds_.size() < LUT_FASTER_THRESHOLD)
        {
            bool firstColor = true;
            for (const auto &item: thresholds_)
            {
                const auto offset = static_cast<uint8_t>(item.first);
                const ColorThreshold& range = item.second;

                if(firstColor)
                {
                    for(uint32_t col = 0; col < uvWidth; col++)
                    {
                        pOutRow[col] = ((pYRow[col] >= range.y[0]) & (pYRow[col] < range.y[1]) & (pURow[col] >= range.u[0]) & (pURow[col] < range.u[1]) & (pVRow[col] >= range.v[0]) & (pVRow[col] < range.v[1])) << offset;
                    }
                    firstColor = false;
                }
                else
                {
                    for(uint32_t col = 0; col < uvWidth; col++)
                    {
                        pOutRow[col] |= ((pYRow[col] >= range.y[0]) & (pYRow[col] < range.y[1]) & (pURow[col] >= range.u[0]) & (pURow[col] < range.u[1]) & (pVRow[col] >= range.v[0]) & (pVRow[col] < range.v[1])) << offset;
                    }
                }
            }
        }
        else
        {
            for(uint32_t col = 0; col < uvWidth; col++)
            {
                pOutRow[col] = lutY_[pYRow[col]] & lutU_[pURow[col]] & lutV_[pVRow[col]];
            }
        }

        if(getDebugLevel())
        {
            for(uint32_t col = 0; col < uvWidth; col++)
            {
                if(pOutRow[col])
                    jobData.pFrame->drawPoint((int)col*2, (int)row*2, colorMap_[pOutRow[col]]);
            }
        }
    }
}

void ColorClassifierYUV::onColorThresholds(ExtColorThresholds* pThresh)
{
    ColorThreshold thresh = { {pThresh->y[0], pThresh->y[1]}, {pThresh->u[0], pThresh->u[1]}, {pThresh->v[0], pThresh->v[1]} };

    switch(pThresh->colorId)
    {
        case EXT_COLOR_THRESHOLDS_ID_ORANGE:
            updateThreshold(Color::Orange, thresh);
            break;
        case EXT_COLOR_THRESHOLDS_ID_WHITE:
            updateThreshold(Color::White, thresh);
            break;
        case EXT_COLOR_THRESHOLDS_ID_BLACK:
            updateThreshold(Color::Black, thresh);
            break;
    }
}

void ColorClassifierYUV::updateThreshold(Color color, const ColorThreshold& threshold)
{
  thresholds_[color] = threshold;
  colorMap_[(1 << (uint8_t)color)] = ColorYUV(threshold.y[0]/2+threshold.y[1]/2, threshold.u[0]/2+threshold.u[1]/2, threshold.v[0]/2+threshold.v[1]/2);
  updateLut();
}

void ColorClassifierYUV::updateLut()
{
    memset(lutY_, 0, sizeof(lutY_));
    memset(lutU_, 0, sizeof(lutU_));
    memset(lutV_, 0, sizeof(lutV_));

    for(const auto& entry : thresholds_)
    {
        const auto& threshold = entry.second;
        uint8_t shifts = static_cast<uint8_t>(entry.first);
        uint8_t colorBit = (1 << shifts);

        for(uint8_t y = threshold.y[0]; y != threshold.y[1]; y++)
        {
            lutY_[y] |= colorBit;
        }

        for(uint8_t u = threshold.u[0]; u != threshold.u[1]; u++)
        {
            lutU_[u] |= colorBit;
        }

        for(uint8_t v = threshold.v[0]; v != threshold.v[1]; v++)
        {
            lutV_[v] |= colorBit;
        }
    }
}
