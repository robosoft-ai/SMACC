
#include <smacc/common.h>
#include <sr_event_countdown/sr_event_countdown.h>
#include <memory>

namespace smacc
{
namespace state_reactors
{
using namespace smacc::introspection;
SrEventCountdown::SrEventCountdown(int eventCount)
    : eventCount_(eventCount)
{
}

void SrEventCountdown::onInitialized()
{
    for (auto type : eventTypes)
    {
        triggeredEvents[type] = false;
    }
}

void SrEventCountdown::onEventNotified(const std::type_info *eventType)
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

bool SrEventCountdown::triggers()
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

} // namespace state_reactors
} // namespace smacc
