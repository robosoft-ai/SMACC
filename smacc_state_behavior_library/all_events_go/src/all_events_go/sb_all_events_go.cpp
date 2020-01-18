
#include <all_events_go/sb_all_events_go.h>
#include <smacc/common.h>

namespace smacc
{
namespace state_behaviors
{

using namespace smacc::introspection;
void SbAllEventsGo::onInitialized()
{
    for (auto type : eventTypes)
    {
        triggeredEvents[type] = false;
    }
}

void SbAllEventsGo::onEventNotified(const std::type_info *eventType)
{
    ROS_INFO_STREAM("SB ALL RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
    triggeredEvents[eventType] = true;

    for (auto &entry : triggeredEvents)
    {
        ROS_INFO_STREAM(demangleSymbol(entry.first->name()) << " = " << entry.second);
    }
}

bool SbAllEventsGo::triggers()
{
    ROS_INFO("SB All TRIGGERS?");
    for (auto &entry : triggeredEvents)
    {
        if (!entry.second)
            return false;
    }
    ROS_INFO("SB ALL TRIGGERED");
    return true;
}
} // namespace state_behaviors
} // namespace smacc