/**
 * RegionExtractor
 *
 * Region based color segmentation.
 *
 * Original authors:
 * - James Bruce (Original CMVision implementation and algorithms)
 * - Stefan Zickler (SSL-Vision code restructuring, and data structure changes)
 */

#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include <memory>
#include "interface/ProcessingStep.h"

class RegionExtractor : public AProcessingStep
{
public:
    RegionExtractor();

    void execute(State& state) override;

private:
    void encodeRuns(State& state);
    static void connectComponents(State& state, Color color);
    static void extractRegions(State& state, Color color);

    //==== Utility Functions ===========================================//
    // sum of integers over range [x,x+w)
    inline static uint32_t rangeSum(uint32_t x, uint32_t w)
    {
        return (w * (2 * x + w - 1) / 2);
    }

    std::vector<std::vector<ColoredRun>> workerRunlists_;
};
