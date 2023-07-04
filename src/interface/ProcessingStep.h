#pragma once

#include <map>
#include "State.h"

class AProcessingStep
{
public:
    virtual ~AProcessingStep() = default;

    virtual void execute(State& state) = 0;

    void handleCommand(const Command& cmd);

    void setEnabled(bool isEnabled) { isEnabled_ = isEnabled; }
    bool isEnabled() const { return isEnabled_; }
    void setDebugLevel(uint8_t debugLevel) { debugLevel_ = debugLevel; }
    uint8_t getDebugLevel() const { return debugLevel_; }

protected:
    template<class Data>
    void addCommandHandler(uint16_t cmdId, std::function<void(Data*)> processor)
    {
        CommandHandler handler;
        handler.handlerFunc = [=](void* data) { processor((Data*) data); };
        handler.packetSize = sizeof(Data);
        commandHandlers_[cmdId] = handler;
    }

    void addCommandHandler(uint16_t cmdId, std::function<void()> processor);

    void addDefaultStepConfigHandler(uint32_t stepMask);

    void handleStepConfig(const ExtStepConfig& cfg, uint32_t stepMask);

private:
    struct CommandHandler
    {
        std::function<void(void*)> handlerFunc;
        size_t packetSize;
    };

    bool isEnabled_ = false;
    uint8_t debugLevel_ = 0;

    std::map<uint16_t, CommandHandler> commandHandlers_;
};
