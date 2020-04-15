#pragma once

#include <thread>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

#include <smacc/smacc_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

  ClMoveGroup(std::string groupName)
      : moveGroupClientInterface(groupName)
  {
    ros::WallDuration(10.0).sleep();
  }

  virtual ~ClMoveGroup()
  {
  }

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

  void moveToAbsolutePose(geometry_msgs::PoseStamped &targetPose)
  {
    bool success = moveEndEfectorToPose(moveGroupClientInterface,
                                        planningSceneInterface,
                                        targetPose);
  }

  bool moveEndEfectorToPose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                            moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                            geometry_msgs::PoseStamped &targetObjectPose)
  {

    moveGroupInterface.setPlanningTime(1.0);

    ROS_INFO_STREAM("Target End efector Pose: " << targetObjectPose);

    moveGroupInterface.setPoseTarget(targetObjectPose);
    moveGroupInterface.setPoseReferenceFrame("/map");

    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
    bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
    {
      auto executionResult = moveGroupInterface.execute(computedMotionPlan);

      if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_INFO("[ClMoveGroup] relative motion execution succedded");
        this->postEventMotionExecutionSucceded();
      }
      else
      {
        ROS_INFO("[ClMoveGroup] motion execution failed");
        this->postEventMotionExecutionFailed();
      }
    }
    else
    {
      ROS_INFO("[ClMoveGroup] motion execution failed");
      this->postEventMotionExecutionFailed();
    }

    return success;
  }

  void moveRelative(geometry_msgs::Transform &transformOffset)
  {
    auto referenceStartPose = this->moveGroupClientInterface.getCurrentPose();
    tf::Quaternion currentOrientation;
    tf::quaternionMsgToTF(referenceStartPose.pose.orientation, currentOrientation);
    tf::Quaternion desiredRelativeRotation;

    tf::quaternionMsgToTF(transformOffset.rotation, desiredRelativeRotation);

    auto targetOrientation = desiredRelativeRotation * currentOrientation;

    auto targetPose = referenceStartPose;

    tf::quaternionTFToMsg(targetOrientation, targetPose.pose.orientation);
    targetPose.pose.position.x += transformOffset.translation.x;
    targetPose.pose.position.y += transformOffset.translation.y;
    targetPose.pose.position.z += transformOffset.translation.z;

    bool success = moveEndEfectorToPose(moveGroupClientInterface,
                                        planningSceneInterface,
                                        targetPose);
  }

  // keeps the end efector orientation fixed
  void moveRelativeCartesian(geometry_msgs::Vector3 &offset)
  {
    std::vector<geometry_msgs::Pose> waypoints;

    auto referenceStartPose = this->moveGroupClientInterface.getPoseTarget();
    //auto referenceStartPose = this->moveGroupClientInterface.getCurrentPose();
    ROS_INFO_STREAM("RELATIVE MOTION, SOURCE POSE: " << referenceStartPose);
    waypoints.push_back(referenceStartPose.pose); // up and out

    auto targetObjectPose = this->moveGroupClientInterface.getPoseTarget();

    auto endPose = referenceStartPose.pose;

    endPose.position.x += offset.x;
    endPose.position.y += offset.y;
    endPose.position.z += offset.z;

    //target_pose2.position.x -= 0.15;
    waypoints.push_back(endPose); // left

    this->moveGroupClientInterface.setPoseTarget(endPose);

    this->moveGroupClientInterface.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = this->moveGroupClientInterface.computeCartesianPath(waypoints,
                                                                          0.01, // eef_step
                                                                          0.0,  // jump_threshold
                                                                          trajectory);

    if (fraction == -1)
    {
      this->postEventMotionExecutionFailed();
      ROS_INFO("[ClMoveGroup] Absolute motion planning failed. Skipping execution.");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan grasp_pose_plan;

    //grasp_pose_plan.start_state_ = *(moveGroupInterface.getCurrentState());
    grasp_pose_plan.trajectory_ = trajectory;
    auto executionResult = this->moveGroupClientInterface.execute(grasp_pose_plan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("[ClMoveGroup] relative motion execution succedded: fraction %lf", fraction);
      this->postEventMotionExecutionSucceded();
    }
    else
    {
      ROS_INFO("[ClMoveGroup] relative motion execution failed");
      this->postEventMotionExecutionFailed();
    }

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
  }
};
} // namespace cl_movegroup
} // namespace sm_moveit