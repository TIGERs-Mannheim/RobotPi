/*
 * TimeSync.cpp
 *
 *  Created on: 18.12.2020
 *      Author: AndreR
 */

#include "TimeSync.h"
#include <numeric>

void TimeSync::addSample(uint32_t timestampMcUs, int64_t timestampStcUs)
{
    std::lock_guard<std::mutex> lock(updateMutex_);

    const int64_t ucTimestampI64 = timestampMcUs;

    if(timestampMcUs < lastMcTimestamp_)
        timeDiffs_.clear();

    lastMcTimestamp_ = timestampMcUs;

    int64_t diff = timestampStcUs - ucTimestampI64;

    timeDiffs_.push_back(diff);

    while(timeDiffs_.size() > 10)
        timeDiffs_.pop_front();

    int64_t sum = std::accumulate(timeDiffs_.begin(), timeDiffs_.end(), (int64_t)0);
    timeDiff_ = sum / timeDiffs_.size();

    // stc - diff = tUc
    // tUc + diff = stc
}

int64_t TimeSync::mc2stc(uint32_t mc)
{
    std::lock_guard<std::mutex> lock(updateMutex_);

    return (int64_t)mc + timeDiff_;
}

uint32_t TimeSync::stc2mc(int64_t stc)
{
    std::lock_guard<std::mutex> lock(updateMutex_);

    int64_t timestampMc = stc - timeDiff_;

    if(timestampMc < 0)
        timestampMc = (int64_t)UINT32_MAX - timestampMc;

    if(timestampMc > UINT32_MAX)
        timestampMc -= UINT32_MAX;

    return (uint32_t)timestampMc;
}
