/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <tf/transform_listener.h>
#include "cb_move_base_client_behavior_base.h"
#include <smacc/smacc_updatable.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{
class CbUndoPathBackwards2 : public CbMoveBaseClientBehaviorBase, public smacc::ISmaccUpdatable
{
public:
  CbUndoPathBackwards2();
  virtual void onEntry() override;

  virtual void onExit() override;

  virtual void update() override;

private:
  ClMoveBaseZ::Goal goal;
  tf::TransformListener listener;
  cl_move_base_z::Pose* robotPose_;
  std::atomic<bool> goalLinePassed_;
};
}  // namespace cl_move_base_z