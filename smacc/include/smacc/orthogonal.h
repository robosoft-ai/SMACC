#pragma once
#include <smacc/common.h>

namespace smacc
{

class Orthogonal
{
public:
    ISmaccStateMachine *stateMachine_;
    std::shared_ptr<smacc::SmaccSubStateBehavior> currentBehavior;

    void setStateMachine(ISmaccStateMachine *value);

    void setStateBehavior(std::shared_ptr<smacc::SmaccSubStateBehavior> statebehavior);

    void onEntry();

    void onExit();

    virtual std::string getName() const;
};

} // namespace smacc