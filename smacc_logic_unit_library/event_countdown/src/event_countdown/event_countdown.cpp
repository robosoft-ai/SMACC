
#include <smacc/common.h>
#include <event_countdown/event_countdown.h>

namespace smacc
{
LuEventCountdown::LuEventCountdown(int eventCount)
    : eventCount_(eventCount)
{
}

void LuEventCountdown::onInitialized()
{
    for (auto type : eventTypes)
    {
        triggeredEvents[type] = false;
    }
}

void LuEventCountdown::onEventNotified(const std::type_info *eventType)
{
    eventCount_--;
    ROS_INFO_STREAM("LU COUNTDOWN ("<< eventCount_ <<") RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));

    // ROS_INFO_STREAM("LU ALL RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
    // triggeredEvents[eventType] = true;

    // for (auto &entry : triggeredEvents)
    // {
    //     ROS_INFO_STREAM(demangleSymbol(entry.first->name()) << " = " << entry.second);
    // }
}

bool LuEventCountdown::triggers()
{
    if( eventCount_ == 0)
    {
        ROS_INFO_STREAM("LU COUNTDOWN ("<< eventCount_ <<") TRIGGERS!" );
        return true;
    }
    else
    {
        return false;
    }
    

    // ROS_INFO("LU All TRIGGERS?");
    // for (auto &entry : triggeredEvents)
    // {
    //     if (!entry.second)
    //         return false;
    // }
    // ROS_INFO("LU ALL TRIGGERED");
    // return true;
}

} // namespace smacc