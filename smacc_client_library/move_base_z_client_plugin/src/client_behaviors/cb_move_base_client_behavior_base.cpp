/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <move_base_z_client_plugin/client_behaviors/cb_move_base_client_behavior_base.h>

namespace cl_move_base_z
{
void CbMoveBaseClientBehaviorBase::propagateSuccessEvent(ClMoveBaseZ::ResultConstPtr& r)
{
  this->postSuccessEvent();
}
void CbMoveBaseClientBehaviorBase::propagateFailureEvent(ClMoveBaseZ::ResultConstPtr& r)
{
  this->postFailureEvent();
}
}  // namespace cl_move_base_z
