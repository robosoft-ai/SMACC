
#pragma once

#include <smacc/smacc.h>
#include <geometry_msgs/Pose.h>
#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_event_dispatcher.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

namespace move_base_z_client
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

class WaypointNavigator : public smacc::ISmaccComponent
{
public:
  WaypointEventDispatcher waypointsEventDispatcher;

  ClMoveBaseZ *client_;

  WaypointNavigator();

  virtual void initialize(smacc::ISmaccClient *owner) override
  {
    client_ = dynamic_cast<ClMoveBaseZ*>(owner);
  }

  void insertWaypoint(int index, geometry_msgs::Pose &newpose);

  void removeWaypoint(int index);

  void loadWayPointsFromFile(std::string filepath);

  void setWaypoints(const std::vector<geometry_msgs::Pose> &waypoints);

  void setWaypoints(const std::vector<Pose2D> &waypoints);

  void sendNextGoal();

  const std::vector<geometry_msgs::Pose> &getWaypoints() const;

  long getCurrentWaypointIndex() const;

  template <typename TObjectTag, typename TDerived>
  void configureEventSourceTypes()
  {
    waypointsEventDispatcher.initialize<TDerived, TObjectTag>(client_);
  }

private:
  void onGoalReached(ClMoveBaseZ::ResultConstPtr &res);

  std::vector<geometry_msgs::Pose> waypoints_;

  boost::signals2::connection succeddedConnection_;

  int currentWaypoint_;
};
} // namespace smacc