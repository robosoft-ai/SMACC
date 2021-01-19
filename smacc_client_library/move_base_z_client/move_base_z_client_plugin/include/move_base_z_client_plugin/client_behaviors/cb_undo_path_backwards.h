/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <tf/transform_listener.h>
#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{
class CbUndoPathBackwards : public CbMoveBaseClientBehaviorBase
{
  tf::TransformListener listener;

  virtual void onEntry() override;

  virtual void onExit() override;
};
}  // namespace cl_move_base_z