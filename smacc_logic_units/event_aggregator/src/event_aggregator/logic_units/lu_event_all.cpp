
#include <event_aggregator/logic_units/lu_event_all.h>
#include <smacc/common.h>

namespace smacc
{
void LuAllEventsGo::onInitialized()
{
    for (auto type : eventTypes)
    {
        triggeredEvents[type] = false;
    }
}

void LuAllEventsGo::onEventNotified(const std::type_info *eventType)
{
    ROS_INFO_STREAM("LU ALL RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
    triggeredEvents[eventType] = true;

    for (auto &entry : triggeredEvents)
    {
        ROS_INFO_STREAM(demangleSymbol(entry.first->name()) << " = " << entry.second);
    }
}

bool LuAllEventsGo::triggers()
{
    ROS_INFO("LU All TRIGGERS?");
    for (auto &entry : triggeredEvents)
    {
        if (!entry.second)
            return false;
    }
    ROS_INFO("LU ALL TRIGGERED");
    return true;
}

}