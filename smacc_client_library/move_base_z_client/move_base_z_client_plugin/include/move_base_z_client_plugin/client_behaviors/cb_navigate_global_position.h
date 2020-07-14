/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_client_behavior.h>
#include <boost/optional.hpp>
#include <geometry_msgs/Point.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include   <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include   <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>
#include <tf/tf.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

class CbNavigateGlobalPosition : public smacc::SmaccClientBehavior
{
public:
  boost::optional<geometry_msgs::Point> goalPosition;
  boost::optional<float> goalYaw;
  boost::optional<float> yawTolerance;
  boost::optional<float> yawToleranceX;
  boost::optional<float> yawToleranceY;


  CbNavigateGlobalPosition();

  CbNavigateGlobalPosition(float x, float y, float yaw /*radians*/);

  void setGoal(const geometry_msgs::Pose& pose);

  virtual void onEntry();

  // auxiliar function that defines the motion that is requested to the move_base action server
  void execute();

  void readStartPoseFromParameterServer(ClMoveBaseZ::Goal &goal);

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  virtual void onExit() override;

private:
  // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server).
  // this resource can be used from any method in this state
  ClMoveBaseZ *moveBaseClient_;
};
} // namespace cl_move_base_z
