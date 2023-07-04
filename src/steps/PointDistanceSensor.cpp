#include "PointDistanceSensor.h"

PointDistanceSensor::PointDistanceSensor()
{
    cfg_.blackThreshold = 50;
    cfg_.whiteThreshold = 110;
    cfg_.tooWhiteThreshold = 160;
    cfg_.topSkipPercent = 0.3f;
    cfg_.bottomSkipPercent = 0.3f;
    cfg_.centerCoverPercent = 0.1f;

    addDefaultStepConfigHandler(EXT_STEP_MASK_DIST_SENSOR);

    addCommandHandler<ExtPointDistanceSensorConfig>(CMD_EXT_POINT_DIST_SENSOR_CFG, [&] (ExtPointDistanceSensorConfig* pCfg) { cfg_ = *pCfg; });
}

enum DetectedColor
{
    WHITE,
    BLACK,
    OTHER,
};

struct VerticalWalkResult
{
    VerticalWalkResult()
    :height(0), yTop(0), yBottom(0), color(OTHER), x(0), yBottomRel(0) {}

    float height;
    uint32_t yTop;
    uint32_t yBottom;
    DetectedColor color;
    uint32_t x;
    float yBottomRel;
};

void PointDistanceSensor::execute(State& state)
{
    const uint8_t blackThreshold = cfg_.blackThreshold;
    const uint8_t whiteThreshold = cfg_.whiteThreshold;
    const uint8_t tooWhiteThreshold = cfg_.tooWhiteThreshold;
    const float bottomSkipPercent = cfg_.bottomSkipPercent;
    const float topSkipPercent = cfg_.topSkipPercent;
    const float centerCoverPercent = cfg_.centerCoverPercent;

    const uint32_t imgWidth = state.pFrame->getMetadata().width;
    const uint32_t imgHeight = state.pFrame->getMetadata().height;
    const uint8_t* yData = state.pFrame->getYData();

    // define Y region to check for obstacle
    const int32_t yTopLimit = static_cast<int32_t>(imgHeight * topSkipPercent);
    const int32_t yBottomLimit = static_cast<int32_t>(imgHeight * (1.0f-bottomSkipPercent));

    // define columns to check close to center (can me more sophisticated selection later)
    uint32_t centerCoverColumns = static_cast<uint32_t>(imgWidth * centerCoverPercent);
    if(centerCoverColumns < 10)
        centerCoverColumns = 10;

    std::vector<uint32_t> testColumns;
    testColumns.reserve(centerCoverColumns);

    for(uint32_t i = 0; i < centerCoverColumns; i++)
    {
        testColumns.push_back(imgWidth/2 - centerCoverColumns/2 + i);
    }

    state.camera.setCurrentResolutionWidth(state.uvWidth);

    // Run up center columns from bottom and store largest same color patch
    std::vector<VerticalWalkResult> verticalWalks(centerCoverColumns);

    for(uint32_t i = 0; i < testColumns.size(); i++)
    {
        const uint32_t x = testColumns[i];

        VerticalWalkResult& res = verticalWalks.at(i);
        res.color = OTHER;
        res.x = x;

        uint32_t bestHeight = 0;

        uint32_t bottomStartRow = 0;
        uint32_t topEndRow = 0;

        DetectedColor lastDetectedColor = OTHER;

        for(int32_t y = yBottomLimit; y > yTopLimit; y--)
        {
            uint8_t pix = yData[y * imgWidth + x];

            DetectedColor detectedColor;
            if(pix < blackThreshold)
                detectedColor = BLACK;
            else if(pix > whiteThreshold && pix < tooWhiteThreshold)
                detectedColor = WHITE;
            else
                detectedColor = OTHER;

            if(lastDetectedColor == OTHER && (detectedColor == WHITE || detectedColor == BLACK))
            {
                // mark start row (from bottom)
                bottomStartRow = y;
            }
            else if((lastDetectedColor == WHITE || lastDetectedColor == BLACK) && detectedColor == OTHER)
            {
                // end row
                topEndRow = y;

                uint32_t height = bottomStartRow - topEndRow;

                if(height > bestHeight)
                {
                    bestHeight = height;
                    res.height = (float)height / imgHeight;
                    res.yBottom = bottomStartRow;
                    res.yBottomRel = (float)bottomStartRow / imgHeight;
                    res.yTop = topEndRow;
                    res.color = lastDetectedColor;
                }
            }
            else
            {
                // same color
            }

            lastDetectedColor = detectedColor;
        }

        if(lastDetectedColor == WHITE || lastDetectedColor == BLACK)
        {
            // end row
            topEndRow = yTopLimit;

            uint32_t height = bottomStartRow - topEndRow;

            if(height > bestHeight)
            {
                bestHeight = height;
                res.height = (float)height / imgHeight;
                res.yBottom = bottomStartRow;
                res.yBottomRel = (float)bottomStartRow / imgHeight;
                res.yTop = topEndRow;
                res.color = lastDetectedColor;
            }
        }
    }

    uint32_t validColumns = 0;
    uint32_t blackColumns = 0;
    uint32_t whiteColumns = 0;
    float heightSum = 0.0f;
    float topSum = 0.0f;
    float bottomSum = 0.0f;

    for(const auto& walk : verticalWalks)
    {
        if(walk.color == WHITE)
            whiteColumns++;

        if(walk.color == BLACK)
            blackColumns++;

        if(walk.color != OTHER)
        {
            validColumns++;

            heightSum += walk.height;
            topSum += walk.yTop;
            bottomSum += walk.yBottomRel;
        }

        if(getDebugLevel() > 1)
        {
            if(walk.color == WHITE)
                state.pFrame->drawLine(walk.x, walk.yTop, walk.x, walk.yBottom, ColorRGB(0.0f, 1.0f, 0.0f));

            if(walk.color == BLACK)
                state.pFrame->drawLine(walk.x, walk.yTop, walk.x, walk.yBottom, ColorRGB(1.0f, 0.0f, 0.0f));
        }
    }

    if(getDebugLevel())
    {
        state.pFrame->drawLine(0, yTopLimit, imgWidth, yTopLimit, ColorRGB(1.0f, 0.3f, 0.3f));
        state.pFrame->drawLine(0, yBottomLimit, imgWidth, yBottomLimit, ColorRGB(0.7f, 0.3f, 0.3f));
    }

    // Construct commands
    const uint32_t ucTime = state.timeSync.stc2mc(state.pFrame->getMetadata().timestampUs - state.pFrame->getMetadata().exposureUs);

    ExtPointDistSensor dist;
    dist.timestampUs = ucTime;
    dist.validColumns = validColumns;
    dist.isMostlyWhite = 0;
    dist.avgHeight = 0.0f;
    dist.avgYBottom = 0.0f;

    if(validColumns)
    {
        dist.avgHeight = heightSum / validColumns;
        dist.avgYBottom = bottomSum / validColumns;

        if(blackColumns > whiteColumns)
            dist.isMostlyWhite = 0;
        else
            dist.isMostlyWhite = 1;
    }

    state.pComm->write({CMD_EXT_POINT_DIST_SENSOR, dist});
}
