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
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior onEntry", this->getName().c_str());
    }

    void SmaccClientBehavior::runtimeConfigure()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior runtimeConfigure", this->getName().c_str());
    }

    void SmaccClientBehavior::onExit()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior onExit", this->getName().c_str());
    }
} // namespace smacc