#include <smacc_navigation_plugin/waypoints_event_dispatcher.h>

namespace smacc
{
void WaypointEventDispatcher::postWaypointEvent(int index)
{
    postWaypointFn[index % WAYPOINTS_EVENTCOUNT]();
}
} // namespace smacc