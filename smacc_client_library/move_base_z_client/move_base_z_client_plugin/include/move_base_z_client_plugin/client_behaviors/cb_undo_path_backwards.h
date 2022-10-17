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
enum UndoPathLocalPlanner
{
  BackwardsLocalPlanner, /*default, it is able to move freely backwards, or combined straightsegments with pure spinning
                            segments*/
  PureSpinningLocalPlanner    /*only pure spinnings, be careful it may not be able to undo the path if it was not a pure
                            spinning segment*/
};

class CbUndoPathBackwards : public CbMoveBaseClientBehaviorBase
{
public:
  boost::optional<UndoPathLocalPlanner> forceUndoLocalPlanner;

  virtual void onEntry() override;

  virtual void onExit() override;

private:
  tf::TransformListener listener;
};
}  // namespace cl_move_base_z
