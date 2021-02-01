/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <boost/optional.hpp>
namespace cl_move_base_z
{
class CbMoveBaseClientBehaviorBase : public smacc::SmaccAsyncClientBehavior
{
public:
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    this->requiresClient(moveBaseClient_);
    moveBaseClient_->onSucceeded(&CbMoveBaseClientBehaviorBase::propagateSuccessEvent, this);
    moveBaseClient_->onAborted(&CbMoveBaseClientBehaviorBase::propagateFailureEvent, this);

    ros::NodeHandle nh;
    visualizationMarkersPub_ = nh.advertise<visualization_msgs::MarkerArray>("move_base_z/visualization_markers", 1);
  }

protected:
  ClMoveBaseZ* moveBaseClient_;
  ros::Publisher visualizationMarkersPub_;

private:
  void propagateSuccessEvent(ClMoveBaseZ::ResultConstPtr& r);
  void propagateFailureEvent(ClMoveBaseZ::ResultConstPtr& r);
};
}  // namespace cl_move_base_z
