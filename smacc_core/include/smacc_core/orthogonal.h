#pragma once
#include <smacc_core/smacc_state_machine.h>
#include <smacc_core/smacc_state.h>

namespace smacc
{
    
class Orthogonal
{

    public:
    ISmaccStateMachine* stateMachine_;
    smacc::SmaccStateBehavior* currentBehavior;

    void setStateMachine(ISmaccStateMachine* value);

    void setStateBehavior(smacc::SmaccStateBehavior* statebehavior);

    void onEntry();

    void onExit();
};

}