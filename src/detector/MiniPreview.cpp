/*
 * MiniPreview.cpp
 *
 *  Created on: 11.11.2020
 *      Author: AndreR
 */

#include "MiniPreview.h"
#include "util/log.h"
#include "commands.h"
#include <string.h>
#include <chrono>

MiniPreview::MiniPreview(uint32_t baudrate)
:enabled_(false), debugLevel_(0), baudrate_(baudrate), transmitting_(false), curTransmitRow_(0)
{
    lastTransmitCompleteTime_ = std::chrono::high_resolution_clock::now();
    lastFrameTime_ = std::chrono::high_resolution_clock::now();
}

Command::List MiniPreview::processFrame(FrameYUV420* pFrame)
{
    Command::List commands;

    if(enabled_)
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
                commands.push_back(Command(CMD_EXT_CAMERA_PREVIEW_LINE_160, line));

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
            if(debugLevel_ > 0)
            {
                float dt = std::chrono::duration<float>(tNow - lastTransmitCompleteTime_).count();
                LogInfo("Creating mini-preview, dt: %.4fs, fps: %.3f, lines: %u\n", dt, 1.0f/(dt), lines);
                LogInfo("Frame DT: %.3f, fps: %.3f\n", frameDt, 1.0f/frameDt);
            }

            lastTransmitCompleteTime_ = tNow;

            createMiniPreview(pFrame);
            transmitting_ = true;
        }
    }
    else
    {
        transmitting_ = false;
        curTransmitRow_ = 0;
    }

    return commands;
}

void MiniPreview::processCommand(std::shared_ptr<Command> pCmd)
{
    if(pCmd->getId() == CMD_EXT_CAMERA_PREVIEW_CONFIG)
    {
        const ExtCameraPreviewConfig* pConfig = pCmd->as<ExtCameraPreviewConfig>();

        if(pConfig)
        {
            enabled_ = pConfig->enable;
        }
    }
}

void MiniPreview::createMiniPreview(const FrameYUV420* pFrame)
{
    uint32_t width = pFrame->getMetadata().width;
    uint32_t height = pFrame->getMetadata().height;

    if((width % PREVIEW_WIDTH) != 0 || (height % PREVIEW_HEIGHT) != 0)
    {
        LogError("Image size not a multiple of preview size\n");
        return;
    }

    uint32_t div = width/PREVIEW_WIDTH;
    uint32_t divH = height/PREVIEW_HEIGHT;

    if(div != divH)
    {
        LogError("Preview must have same aspect ratio as image\n");
        return;
    }

    const uint8_t* pYData = pFrame->getYData();
    const uint8_t* pUData = pFrame->getUData();
    const uint8_t* pVData = pFrame->getVData();

    uint32_t widthUV = width / 2;

    switch(div)
    {
        case 4:
        {
            // Y plane needs average of 4x4
            // U,V planes need average of 2x2
            for(uint32_t row = 0; row < PREVIEW_HEIGHT; row++)
            {
                for(uint32_t col = 0; col < PREVIEW_WIDTH; col++)
                {
                    const uint8_t* pYSrc = &pYData[row*4*width + col*4];
                    const uint8_t* pUSrc = &pUData[row*2*widthUV + col*2];
                    const uint8_t* pVSrc = &pVData[row*2*widthUV + col*2];

                    uint32_t ySum = 0;
                    uint32_t uSum = 0;
                    uint32_t vSum = 0;

                    for(uint32_t srcRow = 0; srcRow < 2; srcRow++)
                    {
                        for(uint32_t srcCol = 0; srcCol < 2; srcCol++)
                        {
                            ySum += (uint32_t)pYSrc[(srcRow*2+0)*width + srcCol*2 + 0]
                                  + (uint32_t)pYSrc[(srcRow*2+0)*width + srcCol*2 + 1]
                                  + (uint32_t)pYSrc[(srcRow*2+1)*width + srcCol*2 + 0]
                                  + (uint32_t)pYSrc[(srcRow*2+1)*width + srcCol*2 + 1];

                            uSum += pUSrc[srcRow*width + srcCol];
                            vSum += pVSrc[srcRow*width + srcCol];
                        }
                    }

                    previewRGB565_[row*PREVIEW_WIDTH + col] = yuv2RGB565(ySum/16, uSum/4, vSum/4);
                }
            }
        }
        break;
        case 8:
        {
            // Y plane needs average of 8x8
            // U,V planes need average of 4x4
            for(uint32_t row = 0; row < PREVIEW_HEIGHT; row++)
            {
                for(uint32_t col = 0; col < PREVIEW_WIDTH; col++)
                {
                    const uint8_t* pYSrc = &pYData[row*8*width + col*8];
                    const uint8_t* pUSrc = &pUData[row*4*widthUV + col*4];
                    const uint8_t* pVSrc = &pVData[row*4*widthUV + col*4];

                    uint32_t ySum = 0;
                    uint32_t uSum = 0;
                    uint32_t vSum = 0;

                    for(uint32_t srcRow = 0; srcRow < 4; srcRow++)
                    {
                        for(uint32_t srcCol = 0; srcCol < 4; srcCol++)
                        {
                            ySum += (uint32_t)pYSrc[(srcRow*2+0)*width + srcCol*2 + 0]
                                  + (uint32_t)pYSrc[(srcRow*2+0)*width + srcCol*2 + 1]
                                  + (uint32_t)pYSrc[(srcRow*2+1)*width + srcCol*2 + 0]
                                  + (uint32_t)pYSrc[(srcRow*2+1)*width + srcCol*2 + 1];

                            uSum += pUSrc[srcRow*width + srcCol];
                            vSum += pVSrc[srcRow*width + srcCol];
                        }
                    }

                    previewRGB565_[row*PREVIEW_WIDTH + col] = yuv2RGB565(ySum/64, uSum/16, vSum/16);
                }
            }
        }
        break;
        case 16:
        {
            for(uint32_t row = 0; row < PREVIEW_HEIGHT; row++)
            {
                for(uint32_t col = 0; col < PREVIEW_WIDTH; col++)
                {
                    const uint8_t* pYSrc = &pYData[row*16*width + col*16];
                    const uint8_t* pUSrc = &pUData[row*8*widthUV + col*8];
                    const uint8_t* pVSrc = &pVData[row*8*widthUV + col*8];

                    uint32_t ySum = 0;
                    uint32_t uSum = 0;
                    uint32_t vSum = 0;

                    for(uint32_t srcRow = 0; srcRow < 8; srcRow++)
                    {
                        for(uint32_t srcCol = 0; srcCol < 8; srcCol++)
                        {
                            ySum += (uint32_t)pYSrc[(srcRow*2+0)*width + srcCol*2 + 0]
                                  + (uint32_t)pYSrc[(srcRow*2+0)*width + srcCol*2 + 1]
                                  + (uint32_t)pYSrc[(srcRow*2+1)*width + srcCol*2 + 0]
                                  + (uint32_t)pYSrc[(srcRow*2+1)*width + srcCol*2 + 1];

                            uSum += pUSrc[srcRow*width + srcCol];
                            vSum += pVSrc[srcRow*width + srcCol];
                        }
                    }

                    previewRGB565_[row*PREVIEW_WIDTH + col] = yuv2RGB565(ySum/256, uSum/64, vSum/64);
                }
            }
        }
        break;
        default:
        {
            LogError("Unsupported division ratio %u\n", div);
        }
        break;
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
