#include <smacc/smacc_client_behavior.h>

namespace smacc
{
SmaccClientBehavior::SmaccClientBehavior()
{
    stateMachine_ = nullptr;
    currentState = nullptr;
}

SmaccClientBehavior::~SmaccClientBehavior()
{
    ROS_WARN("Client behavior deallocated.");
}

std::string SmaccClientBehavior::getName() const
{
    return demangleSymbol(typeid(*this).name());
}

void SmaccClientBehavior::onEntry()
{
    ROS_INFO("SmaccClientBehavior %s onEntry", this->getName().c_str());
}

void SmaccClientBehavior::onExit()
{
    ROS_INFO("SmaccClientBehavior %s onExit", this->getName().c_str());
}
} // namespace smacc