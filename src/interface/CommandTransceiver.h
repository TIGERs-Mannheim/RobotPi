#pragma once

#include <memory>
#include "Command.h"

class CommandTransceiver
{
public:
    virtual ~CommandTransceiver() = default;

    virtual void write(const Command& pCmd) {}

    virtual std::shared_ptr<Command> read() { return nullptr; }
};
