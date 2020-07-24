#include <smacc/smacc_client_behavior.h>

namespace smacc
{
    ISmaccClientBehavior::ISmaccClientBehavior()
    {
        stateMachine_ = nullptr;
        currentState = nullptr;
    }

    ISmaccClientBehavior::~ISmaccClientBehavior()
    {
        ROS_WARN("Client behavior deallocated.");
    }

    std::string ISmaccClientBehavior::getName() const
    {
        return demangleSymbol(typeid(*this).name());
    }

    void ISmaccClientBehavior::runtimeConfigure()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior runtimeConfigure", this->getName().c_str());
    }

    void ISmaccClientBehavior::executeOnEntry()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior onEntry", this->getName().c_str());
        this->onEntry();
    }

    void ISmaccClientBehavior::executeOnExit()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior onExit", this->getName().c_str());
        this->onExit();
    }

    void ISmaccClientBehavior::dispose()
    {

    }
    
} // namespace smacc