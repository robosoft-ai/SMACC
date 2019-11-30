#include <smacc/smacc_substate_behavior.h>

namespace smacc
{
SmaccSubStateBehavior::SmaccSubStateBehavior()
{
    stateMachine = nullptr;
    currentState = nullptr;
}

SmaccSubStateBehavior::~SmaccSubStateBehavior()
{
    ROS_WARN("Substate behavior deallocated.");
}

std::string SmaccSubStateBehavior::getName() const
{
    return demangleSymbol(typeid(*this).name());
}

void SmaccSubStateBehavior::onEntry()
{
    ROS_INFO("SmaccSubStateBehavior %s onEntry", this->getName().c_str());
}

void SmaccSubStateBehavior::onExit()
{
    ROS_INFO("SmaccSubStateBehavior %s onExit", this->getName().c_str());
}
} // namespace smacc