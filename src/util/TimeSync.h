/*
 * TimeSync.h
 *
 *  Created on: 18.12.2020
 *      Author: AndreR
 */

#pragma once

#include <cstdint>
#include <deque>
#include <mutex>

class TimeSync
{
public:
    void addSample(uint32_t timestampMcUs, int64_t timestampStcUs);

    int64_t mc2stc(uint32_t mc);
    uint32_t stc2mc(int64_t stc);

private:
    uint32_t lastMcTimestamp_;
    std::deque<int64_t> timeDiffs_;
    int64_t timeDiff_;
    std::mutex updateMutex_;
};
