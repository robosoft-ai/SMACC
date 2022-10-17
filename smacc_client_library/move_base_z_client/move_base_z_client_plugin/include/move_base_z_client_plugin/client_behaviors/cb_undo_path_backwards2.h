/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <smacc/smacc_updatable.h>
#include <tf/transform_listener.h>
#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{
template <typename TSource, typename TObjectTag>
struct EvGoalVirtualLinePassed : sc::event<EvGoalVirtualLinePassed<TSource, TObjectTag>>
{
};

class CbUndoPathBackwards2 : public CbMoveBaseClientBehaviorBase, public smacc::ISmaccUpdatable
{
public:
  CbUndoPathBackwards2();
  virtual void onEntry() override;

  virtual void onExit() override;

  virtual void update() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    CbMoveBaseClientBehaviorBase::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    postVirtualLinePassed_ = [=] { this->postEvent<EvGoalVirtualLinePassed<TSourceObject, TOrthogonal>>(); };
  }

private:
  void publishMarkers();
  float evalPlaneSide(const geometry_msgs::Pose& pose);

  ClMoveBaseZ::Goal goal_;
  odom_tracker::OdomTracker* odomTracker_;

  tf::TransformListener listener;
  cl_move_base_z::Pose* robotPose_;
  std::atomic<bool> goalLinePassed_;
  float initial_plane_side_;
  float triggerThreshold_;  // meters absolute
  std::function<void()> postVirtualLinePassed_;
};
}  // namespace cl_move_base_z
