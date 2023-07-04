#pragma once
#include <vector>
#include <cstdint>
#include "WorkerPool.h"

class Presummer
{
public:
    Presummer(uint16_t width, uint16_t height);

    void setOutputSize(uint16_t width, uint16_t height);

    void presum(WorkerPool& pool, const uint8_t* inFrame, uint16_t inWidth, uint16_t inHeight);

    const uint8_t* getOutput();

private:
    uint16_t width_;
    uint16_t height_;

    std::vector<uint8_t> intermediate_;
    std::vector<uint8_t> output_;
};
