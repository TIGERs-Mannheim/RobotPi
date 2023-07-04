#pragma once

#include "CommandTransceiver.h"
#include "FrameYUV420.h"
#include "util/Camera.h"
#include "util/EnumIterator.h"
#include "util/TimeSync.h"
#include "util/WorkerPool.h"

#include <map>

enum class Color
{
    Orange,
    White,
    Black
};
typedef EnumIterator<Color, Color::Orange, Color::Black> ColorIterator;

struct ColoredRun
{
    uint32_t x;         // location...
    uint32_t y;         // ...
    uint32_t width;     // ...and width of run
    uint8_t color;      // which color(s) this run represents
    uint32_t parent;    // parent run in run list
};

struct ColoredRegion
{
    // id of the color
    uint8_t color;

    // bounding box (x1,y1) - (x2,y2)
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;

    // centroid
    float cen_x;
    float cen_y;

    // occupied area in pixels
    uint32_t area;
};

struct State
{
    std::unique_ptr<CommandTransceiver> pComm;
    TimeSync timeSync;
    WorkerPool threadPool;
    Camera camera;

    FrameYUV420* pFrame;
    uint16_t uvWidth;
    uint16_t uvHeight;

    const uint8_t* pFrameYDataUVSized;

    std::vector<uint8_t> classifiedFrameData;

    std::map<Color, std::vector<ColoredRun>> frameColoredRuns;
    std::map<Color, std::vector<ColoredRegion>> frameColoredRegions;
};
