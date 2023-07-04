#include "ProcessingStep.h"

void AProcessingStep::handleCommand(const Command& cmd)
{
    auto iter = commandHandlers_.find(cmd.getId());
    if(iter == commandHandlers_.end())
        return;

    if(cmd.getLength() < iter->second.packetSize)
        return;

    iter->second.handlerFunc((void*)cmd.getData());
}

void AProcessingStep::addCommandHandler(uint16_t cmdId, std::function<void()> processor)
{
    CommandHandler handler;
    handler.handlerFunc = [=](void* data) { processor(); };
    handler.packetSize = 0;
    commandHandlers_[cmdId] = handler;
}

void AProcessingStep::addDefaultStepConfigHandler(uint32_t stepMask)
{
    addCommandHandler<ExtStepConfig>(CMD_EXT_STEP_CONFIG, [=] (ExtStepConfig* pCfg) {
        handleStepConfig(*pCfg, stepMask);
    });
}

void AProcessingStep::handleStepConfig(const ExtStepConfig& cfg, uint32_t stepMask)
{
    setEnabled(cfg.enabledSteps & stepMask);

    if(cfg.debugSteps & stepMask)
        setDebugLevel(cfg.debugLevel);
    else
        setDebugLevel(0);
}
