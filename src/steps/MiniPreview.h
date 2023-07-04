/*
 * MiniPreview.h
 *
 *  Created on: 11.11.2020
 *      Author: AndreR
 */

#pragma once

#include "interface/FrameYUV420.h"
#include "util/WorkerPool.h"
#include "util/Presummer.h"
#include "interface/ProcessingStep.h"

#include <chrono>
#include <memory>

class MiniPreview : public AProcessingStep
{
public:
    explicit MiniPreview(uint32_t baudrate);

    void execute(State& state) override;

private:
    void createMiniPreview(WorkerPool& pool, const FrameYUV420* pFrame, const uint8_t* pYPresummed);
    uint16_t yuv2RGB565(uint8_t y, uint8_t u, uint8_t v);

    const uint32_t baudrate_;

    bool transmitting_;
    uint32_t curTransmitRow_;

    std::chrono::high_resolution_clock::time_point lastTransmitCompleteTime_;

    std::chrono::high_resolution_clock::time_point lastFrameTime_;

    static const uint32_t PREVIEW_WIDTH = 160;
    static const uint32_t PREVIEW_HEIGHT = 120;

    Presummer yPresummer_;
    Presummer uPresummer_;
    Presummer vPresummer_;

    uint16_t previewRGB565_[PREVIEW_WIDTH*PREVIEW_HEIGHT];
};
