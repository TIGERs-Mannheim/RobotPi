#include <iostream>
#include <list>
#include "RLEFrame.h"
#include "util/VarInt.h"

/* Run length encoding format description
 *
 * 18 byte header (little-endian): for definition see struct RLEHeader, current version 1
 *
 * for each color:
 *   for each run in color (consecutive runs with the same color are concatenated to one run):
 *      LEB128 length of empty space to previous run
 *      LEB128 length of run
 *   if last run end < width*height:
 *      LEB128 width*height - last run end
 *      LEB128 0
 *
 */

#pragma pack(push,1)
struct RLEHeader
{
    uint8_t version;
    uint8_t colorAmount;
    uint16_t width;
    uint16_t height;
    uint32_t exposureUs;
    int64_t timestampUs;
};
#pragma pack(pop)

RLEFrame::RLEFrame(const FrameMetadata& metadata, const std::map<Color, std::vector<ColoredRun>>& coloredRuns)
{
    pack(metadata, coloredRuns);
}

RLEFrame::RLEFrame(const uint8_t* pData, uint32_t size)
{
    data_.assign(pData, pData+size);
}

void RLEFrame::pack(const FrameMetadata& metadata, const std::map<Color, std::vector<ColoredRun>>& coloredRuns)
{
    data_.resize(sizeof(RLEHeader));
    RLEHeader header {1, (uint8_t)ColorIterator::enumSize(), (uint16_t)(metadata.width/2), (uint16_t)(metadata.height/2), metadata.exposureUs, metadata.timestampUs};
    *((RLEHeader*)data_.data()) = header;

    const uint16_t width = header.width;
    const uint16_t height = header.height;
    const uint32_t size = width*height;

    for(Color color : ColorIterator())
    {
        const auto& runs = coloredRuns.at(color);
        data_.reserve(data_.size() + runs.size() * 2); // Size estimate

        uint32_t pos = 0;
        auto runIt = runs.cbegin();
        while(runIt != runs.cend())
        {
            const ColoredRun& run = *runIt;
            const uint32_t runPos = run.x + run.y * width;

            uint32_t spaceLength = runPos - pos;
            encodeVarUInt32(data_, spaceLength);
            pos += spaceLength;

            uint32_t runEnd = runPos + run.width;
            while(++runIt != runs.cend())
            {
                const ColoredRun& next = *runIt;

                const uint32_t nextPos = next.x + next.y * width;
                if(nextPos >= runEnd)
                    break;

                runEnd += next.width;
            }

            uint32_t runLength = runEnd - pos;
            encodeVarUInt32(data_, runLength);
            pos += runLength;
        }

        if(pos < size)
        {
            encodeVarUInt32(data_, size - pos);
            encodeVarUInt32(data_, 0);
        }
    }
}

struct DecodeRun
{
    uint32_t pos;
    uint32_t length;
};

static void unpackSingleColorRuns(const uint32_t size, const std::vector<uint8_t>::const_iterator& end, std::vector<uint8_t>::const_iterator& itr, std::vector<DecodeRun>& decodeRuns, bool& isTruncated)
{
    decodeRuns.clear();
    uint32_t pos = 0;

    while(pos < size)
    {
        uint32_t length;
        if(!decodeVarUInt32(itr, end, length))
        {
            isTruncated = true;
            return;
        }
        pos += length;

        if(!decodeVarUInt32(itr, end, length))
        {
            isTruncated = true;
            return;
        }

        decodeRuns.push_back({pos, length});
        pos += length;
    }
}

void RLEFrame::unpack(FrameMetadata& metadata, std::map<Color, std::vector<ColoredRun>>& coloredRuns, bool& isTruncated) const
{
    if(!hasValidHeader())
    {
        std::cerr << "Erroneous RLEFrame.unpack attempted" << std::endl;
        return;
    }

    const uint8_t* ptr = data_.data();
    const RLEHeader header = *((RLEHeader*) ptr);
    metadata.timestampUs = header.timestampUs;
    metadata.exposureUs = header.exposureUs;
    metadata.width = header.width*2;
    metadata.height = header.height*2;

    if(header.version != 1)
    {
        std::cerr << "RLEFrame.unpack with unknown version " << header.version << " attempted" << std::endl;
        isTruncated = true;
        return;
    }

    coloredRuns.clear();
    isTruncated = false;

    std::vector<DecodeRun> decodeRuns;
    uint8_t remainingColors = header.colorAmount;
    auto itr = data_.cbegin() + sizeof(RLEHeader);

    for(Color color : ColorIterator())
    {
        if(!remainingColors--)
            break;

        unpackSingleColorRuns(header.width*header.height, data_.cend(), itr, decodeRuns, isTruncated);

        //split line breaks
        const uint8_t colorId = 1 << static_cast<int>(color);
        auto& runs = coloredRuns[color];
        for(DecodeRun& run : decodeRuns)
        {
            uint32_t x = run.pos % header.width;
            uint32_t y = run.pos / header.width;
            while(x + run.length > header.width)
            {
                const uint32_t runWidth = header.width - x;
                runs.push_back({x, y, runWidth, colorId, 0});
                x = 0;
                y++;
                run.length -= runWidth;
            }

            runs.push_back({x, y, run.length, colorId, 0});
        }
    }

    if(remainingColors)
        std::cerr << "RLEFrame.unpack contains more colors than defined" << std::endl;
}

bool RLEFrame::hasValidHeader() const
{
    return data_.size() < sizeof(RLEHeader);
}
