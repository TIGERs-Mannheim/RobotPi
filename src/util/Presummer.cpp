#include "Presummer.h"

Presummer::Presummer(uint16_t width, uint16_t height)
{
    setOutputSize(width, height);
}

void Presummer::setOutputSize(uint16_t width, uint16_t height)
{
    width_ = width;
    height_ = height;
}

const uint8_t *Presummer::getOutput()
{
    return output_.data();
}

static void presum(const uint8_t* inFrame, uint8_t* outFrame, const uint16_t inWidth, const uint16_t yStart, const uint16_t yEnd)
{
    const uint16_t outWidth = inWidth/2;

    for(uint16_t y = yStart; y < yEnd; y++)
    {
        const uint8_t* inRow0 = inFrame + (y * 2    ) * inWidth;
        const uint8_t* inRow1 = inFrame + (y * 2 + 1) * inWidth;

        uint8_t* outRow = outFrame + y * outWidth;

        for(uint32_t x = 0; x < outWidth; x++)
        {
            //uint16_t for dumb gcc-8
            outRow[x] = (uint16_t)(inRow0[2 * x] + inRow0[2 * x + 1] + inRow1[2 * x] + inRow1[2 * x + 1]) / 4;
        }
    }
}

void Presummer::presum(WorkerPool& pool, const uint8_t* inFrame, uint16_t inWidth, uint16_t outHeight)
{
    const unsigned int threads = pool.getNumThreads();
    uint16_t outWidth = inWidth;

    do
    {
        outWidth /= 2;
        outHeight /= 2;

        uint32_t outSize = outWidth * outHeight;
        if(output_.size() < outSize)
            output_.resize(outSize);

        uint8_t* outFrame = output_.data();
        uint16_t heightPerThread = outHeight/threads;
        std::vector<std::future<void>> jobs(threads);
        for(uint32_t i = 0; i < threads; i++)
        {
            jobs[i] = pool.run<void>(std::bind(&::presum, inFrame, outFrame, inWidth, heightPerThread*i, heightPerThread*(i+1)));
        }

        pool.waitFor(jobs);

        if(outWidth > width_ && outHeight > height_)
        {
            std::swap(intermediate_, output_);
            inFrame = intermediate_.data();
            inWidth = outWidth;
        }
    } while(outWidth > width_ && outHeight > height_);
}
