#include <smacc/logic_unit.h>
#include <ros/ros.h>

namespace smacc
{
    namespace logic_units
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

    }
} // namespace smacc
