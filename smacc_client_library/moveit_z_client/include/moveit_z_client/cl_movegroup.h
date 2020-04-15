#pragma once

//#include <smacc/smacc_client.h>
//#include <smacc/smacc_signal.h>
#include <smacc/smacc.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>

namespace sm_moveit
{
namespace cl_movegroup
{

template <typename TSource, typename TObjectTag>
struct MoveGroupMotionExecutionSucceded : sc::event<MoveGroupMotionExecutionSucceded<TSource, TObjectTag>>
{
};

template <typename TSource, typename TObjectTag>
struct MoveGroupMotionExecutionFailed : sc::event<MoveGroupMotionExecutionFailed<TSource, TObjectTag>>
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
  std::function<void()> postEventMotionExecutionSucceded;
  std::function<void()> postEventMotionExecutionFailed;

  smacc::SmaccSignal<void()> onSucceded_;
  smacc::SmaccSignal<void()> onFailed_;

public:
  moveit::planning_interface::MoveGroupInterface moveGroupClientInterface;
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  ClMoveGroup(std::string groupName);

  virtual ~ClMoveGroup();

  template <typename TObjectTag, typename TDerived>
  void configureEventSourceTypes()
  {
    postEventMotionExecutionSucceded = [=]() {
      this->onSucceded_();
      this->postEvent<MoveGroupMotionExecutionSucceded<TDerived, TObjectTag>>();
    };

    postEventMotionExecutionFailed = [=]() {
      this->onFailed_();
      this->postEvent<MoveGroupMotionExecutionFailed<TDerived, TObjectTag>>();
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

  void moveToAbsolutePose(geometry_msgs::PoseStamped &targetPose);

  bool moveEndEfectorToPose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                            moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                            geometry_msgs::PoseStamped &targetObjectPose);

  void moveRelative(geometry_msgs::Transform &transformOffset);

  // keeps the end efector orientation fixed
  void moveRelativeCartesian(geometry_msgs::Vector3 &offset);
};
} // namespace cl_movegroup
} // namespace sm_moveit