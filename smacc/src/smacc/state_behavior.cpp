#include <smacc/smacc_state_behavior.h>
#include <ros/ros.h>

namespace smacc
{

StateBehavior::StateBehavior()
{
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

} // namespace smacc
