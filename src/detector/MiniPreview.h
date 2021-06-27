/*
 * MiniPreview.h
 *
 *  Created on: 11.11.2020
 *      Author: AndreR
 */

#pragma once

#include "interface/Detector.h"
#include <chrono>

class MiniPreview : public Detector
{
public:
    MiniPreview(uint32_t baudrate);

    Command::List processFrame(FrameYUV420* pFrame) override;
    void processCommand(std::shared_ptr<Command> pCmd) override;
    void setDebugLevel(uint8_t level) override { debugLevel_ = level; }

private:
    void createMiniPreview(const FrameYUV420* pFrame);
    uint16_t yuv2RGB565(uint8_t y, uint8_t u, uint8_t v);

    bool enabled_;

    uint8_t debugLevel_;
    const uint32_t baudrate_;

    bool transmitting_;
    uint32_t curTransmitRow_;

    std::chrono::high_resolution_clock::time_point lastTransmitCompleteTime_;

    std::chrono::high_resolution_clock::time_point lastFrameTime_;

    static const uint32_t PREVIEW_WIDTH = 160;
    static const uint32_t PREVIEW_HEIGHT = 120;

    uint16_t previewRGB565_[PREVIEW_WIDTH*PREVIEW_HEIGHT];
};
