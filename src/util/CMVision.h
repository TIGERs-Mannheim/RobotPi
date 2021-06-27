/**
 * CMVision
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

class Region;
class RegionList;
class ColorRegionList;
class RunList;

struct ColoredRegion
{
    // id of the color
    uint8_t color;

    // bounding box (x1,y1) - (x2,y2)
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;

    // centroid
    float cen_x;
    float cen_y;

    // occupied area in pixels
    int32_t area;
};

class CMVision
{
public:
    CMVision(int _max_regions = 10000, int _max_runs = 50000, int _max_colors = 64);
    ~CMVision();

    typedef std::vector<std::vector<ColoredRegion>> ColorRegionLUT;
    typedef std::shared_ptr<ColorRegionLUT> ColorRegionLUTPtr;

    ColorRegionLUTPtr extractRegions(const uint8_t* pImage, uint32_t width, uint32_t height, int min_blob_area);

protected:
    int max_regions;
    int max_runs;
    RegionList* reglist;
    ColorRegionList* colorlist;
    RunList* runlist;

    ColorRegionLUTPtr pOutput_;

private:
    void encodeRuns(const uint8_t* pImage, uint32_t width, uint32_t height, RunList* runlist);
    void connectComponents(RunList* runlist);
    void extractRegions(RegionList* reglist, RunList* runlist);
    void separateRegions(ColorRegionList* colorlist, RegionList* reglist, int min_area);

    //==== Utility Functions ===========================================//
    // sum of integers over range [x,x+w)
    inline static int rangeSum(int x, int w)
    {
        return (w * (2 * x + w - 1) / 2);
    }
};
