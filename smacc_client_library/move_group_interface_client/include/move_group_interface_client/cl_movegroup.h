/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

//#include <smacc/smacc_client.h>
//#include <smacc/smacc_signal.h>
#include <smacc/smacc.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>

namespace cl_move_group_interface
{

template <typename TSource, typename TOrthogonal>
struct EvMoveGroupMotionExecutionSucceded : sc::event<EvMoveGroupMotionExecutionSucceded<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvMoveGroupMotionExecutionFailed : sc::event<EvMoveGroupMotionExecutionFailed<TSource, TOrthogonal>>
{
};

/*
moveit_msgs/MoveItErrorCodes
----------------------------
int32 SUCCESS=1
int32 FAILURE=99999
int32 PLANNING_FAILED=-1
int32 INVALID_MOTION_PLAN=-2
int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
int32 CONTROL_FAILED=-4
int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
int32 TIMED_OUT=-6
int32 PREEMPTED=-7
int32 START_STATE_IN_COLLISION=-10
int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
int32 GOAL_IN_COLLISION=-12
int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
int32 GOAL_CONSTRAINTS_VIOLATED=-14
int32 INVALID_GROUP_NAME=-15
int32 INVALID_GOAL_CONSTRAINTS=-16
int32 INVALID_ROBOT_STATE=-17
int32 INVALID_LINK_NAME=-18
int32 INVALID_OBJECT_NAME=-19
int32 FRAME_TRANSFORM_FAILURE=-21
int32 COLLISION_CHECKING_UNAVAILABLE=-22
int32 ROBOT_STATE_STALE=-23
int32 SENSOR_INFO_STALE=-24
int32 NO_IK_SOLUTION=-31
int32 val
*/

class ClMoveGroup : public smacc::ISmaccClient
{
private:
  std::function<void()> postEventMotionExecutionSucceded_;
  std::function<void()> postEventMotionExecutionFailed_;

  smacc::SmaccSignal<void()> onSucceded_;
  smacc::SmaccSignal<void()> onFailed_;

public:
  // this structure contains the default move_group configuration for any arm motion through move_group
  // the client behavior will override these default values in a copy of this object so that their changes
  // are not persinstent. In the other hand, if you change this client configuration, the parameters will be persistent.
  moveit::planning_interface::MoveGroupInterface moveGroupClientInterface;

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  ClMoveGroup(std::string groupName);

  virtual ~ClMoveGroup();

  void postEventMotionExecutionSucceded();
  void postEventMotionExecutionFailed();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postEventMotionExecutionSucceded_ = [=]() {
      this->onSucceded_();
      this->postEvent<EvMoveGroupMotionExecutionSucceded<TSourceObject, TOrthogonal>>();
    };

    postEventMotionExecutionFailed_ = [=]() {
      this->onFailed_();
      this->postEvent<EvMoveGroupMotionExecutionFailed<TSourceObject, TOrthogonal>>();
    };
  }

  template <typename TCallback, typename T>
  boost::signals2::connection onMotionExecutionSuccedded(TCallback callback, T *object)
  {
    return this->getStateMachine()->createSignalConnection(onSucceded_, callback, object);
  }

  template <typename TCallback, typename T>
  boost::signals2::connection onMotionExecutionFailed(TCallback callback, T *object)
  {
    return this->getStateMachine()->createSignalConnection(onFailed_, callback, object);
  }
};
} // namespace cl_move_group_interface
