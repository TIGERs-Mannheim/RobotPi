#include "CMVision.h"
#include <algorithm>
#include <cstdio>

class Run
{
public:
    int x, y, width;    // location and width of run
    uint8_t color;       // which color(s) this run represents
    int parent, next;    // parent run and next run in run list
};

class RunList
{
private:
    Run* runs;
    int max_runs;
    int used_runs;

public:
    RunList(int _max_runs)
    {
        runs = new Run[_max_runs];
        max_runs = _max_runs;
        used_runs = 0;
    }
    ~RunList()
    {
        delete[] runs;
    }

    void setUsedRuns(int runs)
    {
        used_runs = runs;
    }
    int getUsedRuns()
    {
        return used_runs;
    }

    Run* getRunArrayPointer()
    {
        return runs;
    }
    int getMaxRuns()
    {
        return max_runs;
    }
};

class Region
{
public:
    uint8_t color;        // id of the color
    int x1, y1, x2, y2;   // bounding box (x1,y1) - (x2,y2)
    float cen_x, cen_y; // centroid
    int area;          // occupied area in pixels
    int run_start;     // first run index for this region
    int iterator_id;   // id to prevent duplicate hits by an iterator
    Region* next;      // next region in list
    Region* tree_next; // next pointer for use in spatial lookup trees

    // accessor for centroid
    float operator[](int idx) const
    {
        return ((&cen_x)[idx]);
    }

    // width/height accessors
    int width() const
    {
        return (x2 - x1 + 1);
    }
    int height() const
    {
        return (y2 - y1 + 1);
    }
};

class RegionList
{
private:
    Region* regions;
    int max_regions;
    int used_regions;
public:
    RegionList(int _max_regions)
    {
        regions = new Region[_max_regions];
        max_regions = _max_regions;
        used_regions = 0;
    }
    ~RegionList()
    {
        delete[] regions;
    }

    void setUsedRegions(int regions)
    {
        used_regions = regions;
    }
    int getUsedRegions() const
    {
        return used_regions;
    }
    Region* getRegionArrayPointer() const
    {
        return regions;
    }
    int getMaxRegions() const
    {
        return max_regions;
    }
};

class RegionLinkedList
{
protected:
    Region* _first;
    int _num;
public:
    RegionLinkedList()
    {
        reset();
    }
    Region* getInitialElement() const
    {
        return _first;
    }
    int getNumRegions() const
    {
        return _num;
    }
    ;
    void setFront(Region* r)
    {
        _first = r;
    }
    void setNum(int num)
    {
        _num = num;
    }
    void reset()
    {
        _first = 0;
        _num = 0;
    }
    inline void insertFront(Region* r)
    {
        r->next = _first;
        _first = r;
        _num++;
    }
};

class ColorRegionList
{
private:
    RegionLinkedList* color_regions;
    int num_color_regions;

public:
    ColorRegionList(int _num_color_regions)
    {
        color_regions = new RegionLinkedList[_num_color_regions];
        num_color_regions = _num_color_regions;
    }
    ~ColorRegionList()
    {
        delete[] color_regions;
    }
    const RegionLinkedList& getRegionList(int idx) const
    {
        return color_regions[idx];
    }
    RegionLinkedList* getColorRegionArrayPointer() const
    {
        return color_regions;
    }
    int getNumColorRegions() const
    {
        return num_color_regions;
    }
};

void CMVision::encodeRuns(const uint8_t* pImage, uint32_t width, uint32_t height, RunList* runlist)
// Changes the flat array version of the thresholded image into a run
// length encoded version, which speeds up later processing since we
// only have to look at the points where values change.
{
    const int maxRuns = runlist->getMaxRuns();
    Run* pRuns = runlist->getRunArrayPointer();

    Run run;
    run.next = 0;

    int runIndex = 0;

    for(uint32_t y = 0; y < height; ++y)
    {
        const uint8_t* pRow = pImage + y * width;

        run.y = y;

        uint32_t x = 0;
        while(x < width)
        {
            uint8_t currentColor = pRow[x];
            run.x = x;

            // advance x as long as the color is unchanged
            while(x != width && pRow[x] == currentColor)
                x++;

            // end of row or color changed => save active run
            if(currentColor != 0 || x == width)
            {
                run.color = currentColor;
                run.width = x - run.x;
                run.parent = runIndex;
                pRuns[runIndex++] = run;

                if(runIndex >= maxRuns)
                {
                    runlist->setUsedRuns(runIndex);
                    return;
                }
            }
        }
    }

    runlist->setUsedRuns(runIndex);
}

void CMVision::connectComponents(RunList* runlist)
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

    Run* map = runlist->getRunArrayPointer();
    int num = runlist->getUsedRuns();
    int l1, l2;
    Run r1, r2;
    int i, j, s;

    // l2 starts on first scan line, l1 starts on second
    l2 = 0;
    l1 = 1;
    while(map[l1].y == 0)
        l1++; // skip first line

    // Do rest in lock step
    r1 = map[l1];
    r2 = map[l2];
    s = l1;
    while(l1 < num)
    {
        /*
         printf("%6d:(%3d,%3d,%3d) %6d:(%3d,%3d,%3d)\n",
         l1,r1.x,r1.y,r1.width,
         l2,r2.x,r2.y,r2.width);
         */

        if(r1.color == r2.color && r1.color != 0)
        {
            // case 1: r2.x <= r1.x < r2.x + r2.width
            // case 2: r1.x <= r2.x < r1.x + r1.width
            if((r2.x <= r1.x && r1.x < r2.x + r2.width) || (r1.x <= r2.x && r2.x < r1.x + r1.width))
            {
                if(s != l1)
                {
                    // if we didn't have a parent already, just take this one
                    map[l1].parent = r1.parent = r2.parent;
                    s = l1;
                }
                else if(r1.parent != r2.parent)
                {
                    // otherwise union two parents if they are different

                    // find terminal roots of each path up tree
                    i = r1.parent;
                    while(i != map[i].parent)
                        i = map[i].parent;
                    j = r2.parent;
                    while(j != map[j].parent)
                        j = map[j].parent;

                    // union and compress paths; use smaller of two possible
                    // representative indicies to preserve DAG property
                    if(i < j)
                    {
                        map[j].parent = i;
                        map[l1].parent = map[l2].parent = r1.parent = r2.parent = i;
                    }
                    else
                    {
                        map[i].parent = j;
                        map[l1].parent = map[l2].parent = r1.parent = r2.parent = j;
                    }
                }
            }
        }

        // Move to next point where values may change
        i = (r2.x + r2.width) - (r1.x + r1.width);
        if(i >= 0)
            r1 = map[++l1];
        if(i <= 0)
            r2 = map[++l2];
    }

    // Now we need to compress all parent paths
    for(i = 0; i < num; i++)
    {
        j = map[i].parent;
        map[i].parent = map[j].parent;
    }
}

void CMVision::extractRegions(RegionList* reglist, RunList* runlist)
// Takes the list of runs and formats them into a region table,
// gathering the various statistics along the way.  num is the number
// of runs in the rmap array, and the number of unique regions in
// reg[] (bounded by max_reg) is returned.  Implemented as a single
// pass over the array of runs.
{
    int b, i, n, a;
    Run r;
    Region* reg = reglist->getRegionArrayPointer();
    Run* rmap = runlist->getRunArrayPointer();
    int max_reg = reglist->getMaxRegions();
    int num = runlist->getUsedRuns();

    n = 0;

    for(i = 0; i < num; i++)
    {
        if(rmap[i].color != 0)
        {
            r = rmap[i];
            if(r.parent == i)
            {
                // Add new region if this run is a root (i.e. self parented)
                rmap[i].parent = b = n;  // renumber to point to region id
                reg[b].color = r.color;
                reg[b].area = r.width;
                reg[b].x1 = r.x;
                reg[b].y1 = r.y;
                reg[b].x2 = r.x + r.width;
                reg[b].y2 = r.y;
                reg[b].cen_x = rangeSum(r.x, r.width);
                reg[b].cen_y = r.y * r.width;
                reg[b].run_start = i;
                reg[b].iterator_id = i; // temporarily use to store last run
                n++;
                if(n >= max_reg)
                {
                    reglist->setUsedRegions(max_reg);
                    return;
                }
            }
            else
            {
                // Otherwise update region stats incrementally
                b = rmap[r.parent].parent;
                rmap[i].parent = b; // update parent to identify region id
                reg[b].area += r.width;
                reg[b].x2 = std::max(r.x + r.width, reg[b].x2);
                reg[b].x1 = std::min((int)r.x, reg[b].x1);
                reg[b].y2 = r.y; // last set by lowest run
                reg[b].cen_x += rangeSum(r.x, r.width);
                reg[b].cen_y += r.y * r.width;
                // set previous run to point to this one as next
                rmap[reg[b].iterator_id].next = i;
                reg[b].iterator_id = i;
            }
        }
    }

    // calculate centroids from stored sums
    for(i = 0; i < n; i++)
    {
        a = reg[i].area;
        reg[i].cen_x = (float)reg[i].cen_x / a;
        reg[i].cen_y = (float)reg[i].cen_y / a;
        rmap[reg[i].iterator_id].next = 0; // -1;
        reg[i].iterator_id = 0;
        reg[i].x2--; // change to inclusive range
    }

    reglist->setUsedRegions(n);
    return;
}

void CMVision::separateRegions(ColorRegionList* colorlist, RegionList* reglist, int min_area)
// Splits the various regions in the region table a separate list for
// each color.  The lists are threaded through the table using the
// region's 'next' field.  Returns the maximal area of the regions,
// which can be used later to speed up sorting.
{
    Region* p;
    int i; // ,l;
    uint8_t c;
    int area;
    int num_regions = reglist->getUsedRegions();
    Region* reg = reglist->getRegionArrayPointer();
    int num_colors = colorlist->getNumColorRegions();
    RegionLinkedList* color = colorlist->getColorRegionArrayPointer();

    // clear out the region list head table
    for(i = 0; i < num_colors; i++)
    {
        color[i].reset();
    }

    // step over the table, adding successive
    // regions to the front of each list
    for(i = 0; i < num_regions; i++)
    {
        p = &reg[i];
        c = p->color;
        area = p->area;
        if(c >= num_colors)
        {
            printf("Found a color of index %d...but colorlist is only allocated for a max index of %d\n", c, num_colors - 1);
        }
        else
        {
            if(area >= min_area)
            {
                color[c].insertFront(p);
            }
        }
    }
}

CMVision::CMVision(int _max_regions, int _max_runs, int _max_colors)
{
    pOutput_ = std::make_shared<ColorRegionLUT>(_max_colors);

    max_regions = _max_regions;
    max_runs = _max_runs;
    runlist = new RunList(max_runs);
    reglist = new RegionList(max_regions);
    colorlist = new ColorRegionList(_max_colors);
}

CMVision::~CMVision()
{
    delete reglist;
    delete colorlist;
    delete runlist;
}

CMVision::ColorRegionLUTPtr CMVision::extractRegions(const uint8_t* pImage, uint32_t width, uint32_t height, int min_blob_area)
{
    encodeRuns(pImage, width, height, runlist);
    if(runlist->getUsedRuns() == runlist->getMaxRuns())
    {
        printf("Warning: runlength encoder exceeded current max run size of %d\n", runlist->getMaxRuns());
    }
    //Connect the components of the runlength map:
    connectComponents(runlist);

    //Extract Regions from runlength map:
    extractRegions(reglist, runlist);

    if(reglist->getUsedRegions() == reglist->getMaxRegions())
    {
        printf("Warning: extract regions exceeded maximum number of %d regions\n", reglist->getMaxRegions());
    }

    //Separate Regions by colors:
    separateRegions(colorlist, reglist, min_blob_area);

    // clear output regions
    for(auto& regions : *pOutput_)
    {
        regions.clear();
    }

    // convert internal data to result class (clean interface)
    for(int i = 0; i < colorlist->getNumColorRegions(); i++)
    {
        const auto& linkedList = colorlist->getRegionList(i);

        const Region* pReg = linkedList.getInitialElement();
        while(pReg)
        {
            ColoredRegion temp;
            temp.color = pReg->color;
            temp.x1 = pReg->x1;
            temp.x2 = pReg->x2;
            temp.y1 = pReg->y1;
            temp.y2 = pReg->y2;
            temp.cen_x = pReg->cen_x;
            temp.cen_y = pReg->cen_y;
            temp.area = pReg->area;

            (*pOutput_)[i].push_back(temp);

            pReg = pReg->next;
        }
    }

    return pOutput_;
}
