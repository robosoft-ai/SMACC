
#pragma once

#include <smacc/smacc.h>
#include <geometry_msgs/Pose.h>
#include <smacc_navigation_plugin/waypoints_event_dispatcher.h>
#include <smacc_navigation_plugin/move_base_action_client.h>

namespace smacc
{
class SmaccMoveBaseActionClient;

class WaypointNavigator : public ISmaccComponent
{
public:
  WaypointEventDispatcher waypointsEventDispatcher;

  std::vector<geometry_msgs::Pose> waypoints;

  int currentWaypoint;
  SmaccMoveBaseActionClient *client_;

  WaypointNavigator();

  template <typename TDerived, typename TObjectTag>
  void assignToOrthogonal(SmaccMoveBaseActionClient *client);

  void insertWaypoint(int index, geometry_msgs::Pose &newpose);

  void removeWaypoint(int index);

  void loadWayPointsFromFile(std::string filepath);

private:
  void onGoalReached(SmaccMoveBaseActionClient::ResultConstPtr &res);
};
} // namespace smacc