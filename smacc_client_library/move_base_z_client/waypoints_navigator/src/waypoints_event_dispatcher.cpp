#include <waypoints_navigator/waypoints_event_dispatcher.h>

namespace smacc
{
void WaypointEventDispatcher::postWaypointEvent(int index)
{
    auto& fn = postWaypointFn[index % WAYPOINTS_EVENTCOUNT];
    if(fn!=nullptr)
        fn();
}
} // namespace smacc