/*
 * ColorClassifierYUV.cpp
 *
 *  Created on: 22.11.2020
 *      Author: AndreR
 */

#include "ColorClassifierYUV.h"
#include <cstring>
#include "util/log.h"
#include <thread>
#include <future>

ColorClassifierYUV::ColorClassifierYUV()
:colorMap_(256), multithreaded_(true), drawColors_(false)
{
}

ColorClassifierYUV::Output ColorClassifierYUV::process(FrameYUV420* pFrame)
{
    const uint32_t width = pFrame->getMetadata().width;
    const uint32_t height = pFrame->getMetadata().height;
    const uint32_t widthUV = width/2;
    const uint32_t heightUV = height/2;
    const uint32_t numPixelsUV = widthUV*heightUV;

    Output output;
    output.width = widthUV;
    output.height = heightUV;

    if(result_.size() < numPixelsUV)
        result_.resize(numPixelsUV);

    uint8_t* pOut = result_.data();
    output.pColorClass = pOut;

    if(multithreaded_)
    {
        // note: using a thread pool here does not provide any performance benefit
        // so we just keep it simple here by using std::async

        std::vector<std::future<void>> results(4);

        for(uint32_t i = 0; i < 4; i++)
        {
            results[i] = std::async(std::launch::async, [=]{
                processRows(pFrame, heightUV/4*i, heightUV/4*(i+1));
            });
        }
    }
    else
    {
        processRows(pFrame, 0, heightUV);
    }

    return output;
}

void ColorClassifierYUV::processRows(FrameYUV420* pFrame, uint32_t startRow, uint32_t endRow)
{
    const uint32_t width = pFrame->getMetadata().width;
    const uint32_t widthUV = width/2;

    uint8_t* pOut = result_.data();

    uint8_t* pY = pFrame->getYData();
    uint8_t* pU = pFrame->getUData();
    uint8_t* pV = pFrame->getVData();

    for(uint32_t rowUV = startRow; rowUV < endRow; rowUV++)
    {
        uint8_t* pYRow0 = pY + rowUV*2*width;
        uint8_t* pYRow1 = pY + (rowUV*2+1)*width;
        uint8_t* pURow = pU + rowUV*widthUV;
        uint8_t* pVRow = pV + rowUV*widthUV;

        uint8_t* pOutRow = pOut + rowUV*widthUV;

        for(uint32_t col = 0; col < widthUV; col += 4)
        {
            uint8_t y[4];
            y[0] = (pYRow0[0] + pYRow0[1] + pYRow1[0] + pYRow1[1]) / 4;
            y[1] = (pYRow0[2] + pYRow0[3] + pYRow1[2] + pYRow1[3]) / 4;
            y[2] = (pYRow0[4] + pYRow0[5] + pYRow1[4] + pYRow1[5]) / 4;
            y[3] = (pYRow0[6] + pYRow0[7] + pYRow1[6] + pYRow1[7]) / 4;
            pYRow0 += 8;
            pYRow1 += 8;

            pOutRow[0] = lutY_[y[0]] & lutU_[pURow[0]] & lutV_[pVRow[0]];
            pOutRow[1] = lutY_[y[1]] & lutU_[pURow[1]] & lutV_[pVRow[1]];
            pOutRow[2] = lutY_[y[2]] & lutU_[pURow[2]] & lutV_[pVRow[2]];
            pOutRow[3] = lutY_[y[3]] & lutU_[pURow[3]] & lutV_[pVRow[3]];

            if(drawColors_)
            {
                if(pOutRow[0])
                    pFrame->drawPoint(col*2, rowUV*2, colorMap_[pOutRow[0]]);

                if(pOutRow[1])
                    pFrame->drawPoint((col+1)*2, rowUV*2, colorMap_[pOutRow[1]]);

                if(pOutRow[2])
                    pFrame->drawPoint((col+2)*2, rowUV*2, colorMap_[pOutRow[2]]);

                if(pOutRow[3])
                    pFrame->drawPoint((col+3)*2, rowUV*2, colorMap_[pOutRow[3]]);
            }

            pOutRow += 4;

            pURow += 4;
            pVRow += 4;
        }
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
