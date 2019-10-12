#include <smacc/logic_units/logic_unit_base.h>
#include <ros/ros.h>

namespace smacc
{
LogicUnit::LogicUnit()
{
}

void LogicUnit::onInitialized()
{
}

void LogicUnit::onEventNotified(const std::type_info *eventType)
{
}

void LogicUnit::update()
{
    if (this->triggers())
    {
        ROS_INFO("Logic unit base REALLY TRIGGERS!!");
        this->postEventFn();
    }
}

} // namespace smacc
