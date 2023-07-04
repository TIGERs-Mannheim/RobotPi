#pragma once

#include <vector>
#include <memory>
#include "interface/FrameYUV420.h"

class YUVFile
{
public:
    explicit YUVFile(const std::string& path);

    FrameYUV420* getFrame() { return frame_.get(); }

    static void write(const std::string& path, const FrameYUV420* pFrame);

private:
    std::unique_ptr<FrameYUV420> frame_;

    std::vector<uint8_t> y_;
    std::vector<uint8_t> u_;
    std::vector<uint8_t> v_;
};
