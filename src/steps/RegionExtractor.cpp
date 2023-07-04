#include "RegionExtractor.h"
#include <algorithm>

static void encodeRunRows(std::vector<ColoredRun>* workerRunlist, const uint8_t* pImage, uint32_t width, uint32_t startRow, uint32_t endRow)
// Changes the flat array version of the thresholded image into a run
// length encoded version, which speeds up later processing since we
// only have to look at the points where values change.
{
    workerRunlist->clear();

    const uint32_t vecWidth = width - width%8;

    ColoredRun run{0, 0, 0, 0};

    for(uint32_t y = startRow; y < endRow; ++y)
    {
        run.y = y;

        const uint8_t* row = pImage + y * width;
        const uint64_t* vecRow = (uint64_t*)row;

        run.x = 0;
        uint8_t searchValue = 0;
        uint64_t vecSearchValue = 0;
        for(uint32_t vx = 0; vx < vecWidth; vx+=8)
        {
            if(vecRow[vx/8] != vecSearchValue)
            {
                for(uint32_t x = vx; x < vx+8; x++)
                {
                    const uint8_t value = row[x];
                    if(value != searchValue)
                    {
                        if(searchValue)
                        {
                            run.width = x - run.x;
                            run.color = searchValue;
                            workerRunlist->push_back(run);
                        }

                        run.x = x;
                        searchValue = value;
                        vecSearchValue = searchValue * 0x0101010101010101ll;
                    }
                }
            }
        }

        for(uint32_t x = vecWidth; x < width; x++)
        {
            const uint8_t value = row[x];
            if(value != searchValue)
            {
                if(searchValue)
                {
                    run.width = x - run.x;
                    run.color = searchValue;
                    workerRunlist->push_back(run);
                }

                run.x = x;
                searchValue = value;
            }
        }

        if(searchValue)
        {
            run.width = width - run.x;
            run.color = searchValue;
            workerRunlist->push_back(run);
        }
    }
}

void RegionExtractor::encodeRuns(State& state)
{
    const unsigned int threads = state.threadPool.getNumThreads();
    workerRunlists_.resize(threads);

    std::vector<std::future<void>> jobs(threads);
    for(uint32_t i = 0; i < threads; i++)
    {
        auto* pWorkerRunlist = &workerRunlists_[i];

        jobs[i] = state.threadPool.run<void>(std::bind(&encodeRunRows, pWorkerRunlist, state.classifiedFrameData.data(), state.uvWidth, state.uvHeight/threads*i, state.uvHeight/threads*(i+1)));
    }
    state.threadPool.waitFor(jobs);

    for(Color color : ColorIterator())
    {
        auto& colorRuns = state.frameColoredRuns[color];
        colorRuns.clear();

        int colorId = 1 << static_cast<int>(color);

        for(std::vector<ColoredRun>& runs : workerRunlists_)
        {
            for(ColoredRun& run : runs)
            {
                if(run.color & colorId)
                    colorRuns.push_back(run);
            }
        }
    }

}

void RegionExtractor::connectComponents(State& state, Color color)
// Connect components using four-connecteness so that the runs each
// identify the global parent of the connected region they are a part
// of.  It does this by scanning adjacent rows and merging where
// similar colors overlap.  Used to be union by rank w/ path
// compression, but now is just uses path compression as the global
// parent index is a simpler rank bound in practice.
// WARNING: This code is complicated.  I'm pretty sure it's a correct
//   implementation, but minor changes can easily cause big problems.
//   Read the papers on this library and have a good understanding of
//   tree-based union find before you touch it
{
    auto& runs = state.frameColoredRuns[color];
    for(size_t i = 0; i < runs.size(); i++)
        runs[i].parent = i;

    // aboveIndex starts on first scan line, currentIndex starts on second
    unsigned int currentIndex = 1, aboveIndex = 0;

    // Do rest in lock step
    while (currentIndex < runs.size())
    {
        ColoredRun &current = runs[currentIndex];
        ColoredRun &above = runs[aboveIndex];

        if(current.y > above.y + 1)
        {
            aboveIndex++;
            continue;
        }
        else if(current.y == above.y)
        {
            currentIndex++;
            continue;
        }

        uint32_t aboveEnd = above.x + above.width;
        uint32_t currentEnd = current.x + current.width;

        // case 1: above.x <= current.x < aboveEnd
        // case 2: current.x <= above.x < currentEnd
        if ((above.x <= current.x && current.x < aboveEnd) || (current.x <= above.x && above.x < currentEnd))
        {
            if (current.parent == currentIndex)
            {
                // if we didn't have a parent already, just take this one
                current.parent = above.parent;
            } else if (current.parent != above.parent) {
                // otherwise union two parents if they are different

                // find terminal roots of each path up tree
                uint32_t currentParent = current.parent;
                while (currentParent != runs[currentParent].parent)
                    currentParent = runs[currentParent].parent;
                uint32_t aboveParent = above.parent;
                while (aboveParent != runs[aboveParent].parent)
                    aboveParent = runs[aboveParent].parent;

                // union and compress paths; use smaller of two possible
                // representative indicies to preserve DAG property
                uint32_t parent = currentParent < aboveParent ? currentParent : aboveParent;
                runs[currentParent].parent = runs[aboveParent].parent = current.parent = above.parent = parent;
            }
        }

        // Move to next point where values may change
        if (aboveEnd >= currentEnd)
            currentIndex++;
        if (aboveEnd <= currentEnd)
            aboveIndex++;
    }

    // Now we need to compress all parent paths
    for (auto &run : runs) {
        run.parent = runs[run.parent].parent;
    }
}

void RegionExtractor::extractRegions(State& state, Color color)
// Takes the list of runs and formats them into a region table,
// gathering the various statistics along the way.  num is the number
// of runs in the rmap array, and the number of unique regions in
// reg[] (bounded by max_reg) is returned.  Implemented as a single
// pass over the array of runs.
{
    auto& regions = state.frameColoredRegions[color];
    regions.clear();

    auto& runs = state.frameColoredRuns[color];
    int colorId = 1 << static_cast<int>(color);

    for(unsigned int i = 0; i < runs.size(); i++)
    {
        ColoredRun& run = runs[i];

        if(run.parent == i)
        {
            // Add new region
            run.parent = regions.size();  // renumber to point to region id
            ColoredRegion region;
            region.color = colorId;
            region.area = run.width;
            region.x1 = run.x;
            region.y1 = run.y;
            region.x2 = run.x + run.width;
            region.y2 = run.y;
            region.cen_x = rangeSum(run.x, run.width);
            region.cen_y = run.y * run.width;
            regions.push_back(region);
        }
        else
        {
            // Update region
            uint32_t regionId = runs[run.parent].parent;
            run.parent = regionId;
            ColoredRegion& region = regions[regionId];
            region.area += run.width;
            region.x2 = std::max(run.x + run.width, region.x2);
            region.x1 = std::min(run.x, region.x1);
            region.y2 = run.y; // last set by lowest run
            region.cen_x += rangeSum(run.x, run.width);
            region.cen_y += run.y * run.width;
        }
    }

    // calculate centroids from stored sums
    for(ColoredRegion& region : regions)
    {
        region.cen_x /= region.area;
        region.cen_y /= region.area;
        region.x2--; // change to inclusive range
    }
}

RegionExtractor::RegionExtractor()
{
    addDefaultStepConfigHandler(EXT_STEP_MASK_REGION_EXTRACTOR);
}

const static ColorYUV COLOR_TABLE[] = {
        {127, 127, 127}, //
        {127, 000, 255}, //                         orange
        {255, 127, 127}, //                 white
        {191, 000, 255}, //                 white + orange
        {000, 127, 127}, //         black
        {063, 000, 255}, //         black         + orange
        {127, 255, 255}, //         black + white
        {127, 255, 255}, //         black + white + orange
        {127, 000, 000}, // green
        {127, 255, 255}, // green                 + orange
        {191, 000, 000}, // green         + white
        {127, 255, 255}, // green         + white + orange
        {063, 000, 000}, // green + black
        {127, 255, 255}, // green + black         + orange
        {127, 255, 255}, // green + black + white
        {127, 255, 255}, // green + black + white + orange
};

void RegionExtractor::execute(State& state)
{
    encodeRuns(state);

    for(Color color : ColorIterator())
    {
        //Connect the components of the runlength map:
        connectComponents(state, color);

        //Extract Regions from runlength map:
        extractRegions(state, color);
    }

    if(getDebugLevel())
    {
        const size_t colorTableSize = sizeof(COLOR_TABLE)/sizeof(COLOR_TABLE[0]);

        for(const auto& runs : state.frameColoredRuns)
        {
            for(const ColoredRun& run : runs.second)
            {
                if(run.color >= colorTableSize)
                    continue;

                state.pFrame->drawLine((float)run.x*2, (float)run.y*2, (float)(run.x + run.width - 1)*2, (float)run.y*2, COLOR_TABLE[run.color]);
            }
        }

        for(const auto& regions : state.frameColoredRegions)
        {
            for(const ColoredRegion& region : regions.second)
            {
                if(region.color >= colorTableSize)
                    continue;

                state.pFrame->drawRect((float)region.x1*2, (float)region.y1*2, (float)region.x2*2, (float)region.y2*2, COLOR_TABLE[region.color]);
            }
        }
    }
}
