
#pragma once

#include <smacc/smacc.h>
#include <geometry_msgs/Pose.h>
#include <smacc_navigation_plugin/waypoints_event_dispatcher.h>
#include <smacc_navigation_plugin/move_base_action_client.h>

namespace smacc
{
class ClMoveBaseZ;

struct Pose2D
{
  Pose2D(double x, double y, double yaw)
  {
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = yaw;
  }

  double x_;
  double y_;
  double yaw_;
};

class WaypointNavigator : public ISmaccComponent
{
public:
  WaypointEventDispatcher waypointsEventDispatcher;

  ClMoveBaseZ *client_;

  WaypointNavigator();

  template <typename TDerived, typename TObjectTag>
  void assignToOrthogonal(ClMoveBaseZ *client);

  void insertWaypoint(int index, geometry_msgs::Pose &newpose);

  void removeWaypoint(int index);

  void loadWayPointsFromFile(std::string filepath);

  void setWaypoints(const std::vector<geometry_msgs::Pose> &waypoints);

  void setWaypoints(const std::vector<Pose2D> &waypoints);

  void sendNextGoal();

  const std::vector<geometry_msgs::Pose> &getWaypoints() const;

  long getCurrentWaypointIndex() const;

private:
  void onGoalReached(ClMoveBaseZ::ResultConstPtr &res);

  std::vector<geometry_msgs::Pose> waypoints_;

  boost::signals2::connection succeddedConnection_;

  int currentWaypoint_;
};
} // namespace smacc