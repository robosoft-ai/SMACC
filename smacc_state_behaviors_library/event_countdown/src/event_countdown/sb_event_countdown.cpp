
#include <smacc/common.h>
#include <event_countdown/sb_event_countdown.h>
#include <memory>

namespace smacc
{
namespace state_behaviors
{
using namespace smacc::introspection;
SbEventCountdown::SbEventCountdown(int eventCount)
    : eventCount_(eventCount)
{
}

void SbEventCountdown::onInitialized()
{
    for (auto type : eventTypes)
    {
        triggeredEvents[type] = false;
    }
}

void SbEventCountdown::onEventNotified(const std::type_info *eventType)
{
    eventCount_--;
    ROS_INFO_STREAM("SB COUNTDOWN (" << eventCount_ << ") RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));

    // ROS_INFO_STREAM("SB ALL RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
    // triggeredEvents[eventType] = true;

    // for (auto &entry : triggeredEvents)
    // {
    //     ROS_INFO_STREAM(demangleSymbol(entry.first->name()) << " = " << entry.second);
    // }
}

bool SbEventCountdown::triggers()
{
    if (eventCount_ == 0)
    {
        ROS_INFO_STREAM("SB COUNTDOWN (" << eventCount_ << ") TRIGGERS!");
        return true;
    }
    else
    {
        return false;
    }

    // ROS_INFO("SB All TRIGGERS?");
    // for (auto &entry : triggeredEvents)
    // {
    //     if (!entry.second)
    //         return false;
    // }
    // ROS_INFO("SB ALL TRIGGERED");
    // return true;
}

} // namespace state_behaviors
} // namespace smacc