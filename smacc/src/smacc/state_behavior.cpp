#include <smacc/smacc_state_behavior.h>
#include <ros/ros.h>

namespace smacc
{

StateBehavior::StateBehavior()
{
}

void StateBehavior::initialize(smacc::ISmaccState *ownerState)
{
    this->ownerState = ownerState;

    this->onInitialized();
}

void StateBehavior::onInitialized()
{
}

void StateBehavior::onEventNotified(const std::type_info *eventType)
{
}

void StateBehavior::update()
{
    if (this->triggers())
    {
        ROS_INFO("State behavior base REALLY TRIGGERS!!");
        this->postEventFn();
    }
}

namespace introspection
{
void StateBehaviorHandler::configureStateBehavior(std::shared_ptr<smacc::StateBehavior> sb)
{
    for (auto callback : this->callbacks_)
    {
        callback.fn(sb);
    }
}
} // namespace introspection
} // namespace smacc
