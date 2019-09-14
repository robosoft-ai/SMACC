#pragma once
#include <smacc/smacc_state_machine.h>
#include <smacc/smacc_substate_behavior.h>

namespace smacc
{

class Orthogonal
{
public:
    ISmaccStateMachine *stateMachine_;
    smacc::SmaccStateBehavior *currentBehavior;

    void setStateMachine(ISmaccStateMachine *value);

    void setStateBehavior(smacc::SmaccStateBehavior *statebehavior);

    void onEntry();

    void onExit();

    virtual std::string getName() const;
};

} // namespace smacc