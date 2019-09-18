#pragma once
#include <smacc/smacc_state_machine.h>
#include <smacc/smacc_substate_behavior.h>

namespace smacc
{

class Orthogonal
{
public:
    ISmaccStateMachine *stateMachine_;
    smacc::SmaccSubStateBehavior *currentBehavior;

    void setStateMachine(ISmaccStateMachine *value);

    void setStateBehavior(smacc::SmaccSubStateBehavior *statebehavior);

    void onEntry();

    void onExit();

    virtual std::string getName() const;
};

} // namespace smacc