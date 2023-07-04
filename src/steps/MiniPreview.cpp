/*
 * MiniPreview.cpp
 *
 *  Created on: 11.11.2020
 *      Author: AndreR
 */

#include "MiniPreview.h"
#include "util/log.h"
#include "commands.h"
#include <cstring>
#include <chrono>

MiniPreview::MiniPreview(uint32_t baudrate)
: baudrate_(baudrate), transmitting_(false), curTransmitRow_(0), yPresummer_(PREVIEW_WIDTH, PREVIEW_HEIGHT), uPresummer_(PREVIEW_WIDTH, PREVIEW_HEIGHT), vPresummer_(PREVIEW_WIDTH, PREVIEW_HEIGHT)
{
    lastTransmitCompleteTime_ = std::chrono::high_resolution_clock::now();
    lastFrameTime_ = std::chrono::high_resolution_clock::now();

    addCommandHandler<ExtStepConfig>(CMD_EXT_STEP_CONFIG, [&] (ExtStepConfig* pCfg) {
        bool enable = pCfg->enabledSteps & EXT_STEP_MASK_MINI_PREVIEW;
        if(isEnabled() != enable)
        {
            transmitting_ = false;
            curTransmitRow_ = 0;
        }

        handleStepConfig(*pCfg, EXT_STEP_MASK_MINI_PREVIEW);
    });
}

void MiniPreview::execute(State& state)
{
    auto tNow = std::chrono::high_resolution_clock::now();
    float frameDt = std::chrono::duration<float>(tNow - lastFrameTime_).count();
    lastFrameTime_ = tNow;

    float bandwidth = (float)baudrate_ * 0.1f * frameDt;
    uint32_t lines = bandwidth/(sizeof(ExtCameraPreviewLine160)+4);
    if(lines == 0)
        lines = 1;

    if(transmitting_)
    {
        ExtCameraPreviewLine160 line;

        for(uint8_t i = 0; i < lines; i++)
        {
            line.row = curTransmitRow_;
            memcpy(line.data, previewRGB565_ + curTransmitRow_*PREVIEW_WIDTH, PREVIEW_WIDTH*2);
            curTransmitRow_++;
            state.pComm->write({CMD_EXT_CAMERA_PREVIEW_LINE_160, line});

            if(curTransmitRow_ >= PREVIEW_HEIGHT)
            {
                curTransmitRow_ = 0;
                transmitting_ = false;
                break;
            }
        }
    }
    else
    {
        if(getDebugLevel())
        {
            float dt = std::chrono::duration<float>(tNow - lastTransmitCompleteTime_).count();
            LogInfo("Creating mini-preview, dt: %.4fs, fps: %.3f, lines: %u\n", dt, 1.0f/(dt), lines);
            LogInfo("Frame DT: %.3f, fps: %.3f\n", frameDt, 1.0f/frameDt);
        }

        lastTransmitCompleteTime_ = tNow;

        createMiniPreview(state.threadPool, state.pFrame, state.pFrameYDataUVSized);
        transmitting_ = true;
    }
}

void MiniPreview::createMiniPreview(WorkerPool& pool, const FrameYUV420* pFrame, const uint8_t* pYPresummed)
{
    uint32_t width = pFrame->getMetadata().width/2;
    uint32_t height = pFrame->getMetadata().height/2;
    yPresummer_.presum(pool, pYPresummed, width, height);
    uPresummer_.presum(pool, pFrame->getUData(), width, height);
    vPresummer_.presum(pool, pFrame->getVData(), width, height);

    const uint8_t* pYData = yPresummer_.getOutput();
    const uint8_t* pUData = uPresummer_.getOutput();
    const uint8_t* pVData = vPresummer_.getOutput();

    for(uint32_t i = 0; i < PREVIEW_WIDTH*PREVIEW_HEIGHT; i++)
    {
        previewRGB565_[i] = yuv2RGB565(pYData[i], pUData[i], pVData[i]);
    }
}

uint16_t MiniPreview::yuv2RGB565(uint8_t y, uint8_t u, uint8_t v)
{
    float Y = y;
    float U = u;
    float V = v;

    float R = Y + 1.402f * (V - 128);
    float G = Y - 0.34414f * (U - 128) - 0.71414f * (V - 128);
    float B = Y + 1.772f * (U - 128);

    if(R < 0)
        R = 0;
    if(R > 255)
        R = 255;
    if(G < 0)
        G = 0;
    if(G > 255)
        G = 255;
    if(B < 0)
        B = 0;
    if(B > 255)
        B = 255;

    uint16_t r16 = (uint16_t)R;
    uint16_t g16 = (uint16_t)G;
    uint16_t b16 = (uint16_t)B;

    return ((r16 & 0xF8) << 8) | ((g16 & 0xFC) << 3) | (b16 >> 3);
}
