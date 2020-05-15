#include <move_base_z_client_plugin/client_behaviors/cb_navigate_next_waypoint.h>
#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_navigator.h>

namespace cl_move_base_z
{
    CbNavigateNextWaypoint::CbNavigateNextWaypoint()
    {
    }

    CbNavigateNextWaypoint::~CbNavigateNextWaypoint()
    {

    }

    void CbNavigateNextWaypoint::onEntry()
    {
        ClMoveBaseZ *move_base;
        this->requiresClient(move_base);

        auto waypointsNavigator = move_base->getComponent<WaypointNavigator>();
        waypointsNavigator->sendNextGoal();
        ROS_INFO("[CbNavigateNextWaypoint] current iteration waypoints x: %ld", waypointsNavigator->getCurrentWaypointIndex());
    }

    void CbNavigateNextWaypoint::onExit()
    {
    }

} // namespace cl_move_base_z
