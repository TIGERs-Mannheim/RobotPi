#pragma once

#include <cstdint>
#include "steps/RegionExtractor.h"
#include "steps/ColorClassifierYUV.h"

class RLEFrame
{
public:
    RLEFrame() = default;
    RLEFrame(const FrameMetadata& metadata, const std::map<Color, std::vector<ColoredRun>>& coloredRuns);
    RLEFrame(const uint8_t* data, uint32_t size);

    void pack(const FrameMetadata& metadata, const std::map<Color, std::vector<ColoredRun>>& coloredRuns);
    void unpack(FrameMetadata& metadata, std::map<Color, std::vector<ColoredRun>>& coloredRuns, bool& isTruncated) const;

    bool hasValidHeader() const;

    const std::vector<uint8_t>& getData() const { return data_; }
private:
    std::vector<uint8_t> data_;
};
