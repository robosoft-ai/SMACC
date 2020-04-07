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
class ClMoveGroup : public smacc::ISmaccClient
{
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

  void moveToAbsolutePose(geometry_msgs::PoseStamped &targetPose)
  {
    bool success = moveToObjectGraspPose(moveGroupClientInterface,
                                         planningSceneInterface,
                                         targetPose);

    if (success)
    {
      nextMotion(moveGroupClientInterface,
                 planningSceneInterface,
                 targetPose);
    }
  }

  bool moveToObjectGraspPose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
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
      moveGroupInterface.execute(computedMotionPlan);
      ros::WallDuration(15.0).sleep();
    }

    return success;
  }

  void nextMotion(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                  moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                  geometry_msgs::PoseStamped &targetObjectPose)
  {

    ROS_INFO("------- CARTESIAN TEST --------");
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(targetObjectPose.pose); // up and out
    auto target_pose2 = targetObjectPose.pose;

    target_pose2.position.z -= 0.12;
    //target_pose2.position.x -= 0.15;
    waypoints.push_back(target_pose2); // left

    moveGroupInterface.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = moveGroupInterface.computeCartesianPath(waypoints,
                                                              0.01, // eef_step
                                                              0.0,  // jump_threshold
                                                              trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan grasp_pose_plan;
    ;
    //grasp_pose_plan.start_state_ = *(moveGroupInterface.getCurrentState());
    grasp_pose_plan.trajectory_ = trajectory;
    moveGroupInterface.execute(grasp_pose_plan);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

    ros::WallDuration(5.0).sleep();
    ROS_INFO("------- RETREAT TEST --------");
    /*planningSceneInterface.removeCollisionObjects({"cube_0"});
    std::vector<double> group_variable_values;
    moveGroupInterface.getCurrentState()->copyJointGroupPositions(moveGroupInterface.getCurrentState()->getRobotModel()->getJointModelGroup(moveGroupInterface.getName()), group_variable_values);
    group_variable_values[group_variable_values.size() -1 ]=0;
    group_variable_values[group_variable_values.size() -2 ]=0;
  
    
    success = (moveGroupInterface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if(!success)
      exit(0);*/

    waypoints.clear();

    target_pose2 = targetObjectPose.pose;
    target_pose2.position.z -= 0.12;
    waypoints.push_back(target_pose2);          // up and out
    waypoints.push_back(targetObjectPose.pose); // up and out

    moveGroupInterface.setMaxVelocityScalingFactor(0.1);

    planningSceneInterface.removeCollisionObjects({"cube_0"});
    moveit_msgs::RobotTrajectory retreat_trajectory;
    fraction = moveGroupInterface.computeCartesianPath(waypoints,
                                                       0.01, // eef_step
                                                       0.0,  // jump_threshold
                                                       retreat_trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    retreat_plan.trajectory_ = retreat_trajectory;
    moveGroupInterface.execute(retreat_plan);

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);

    ROS_INFO("------- ENDING --------");
    // ros::WallDuration(15.0).sleep();
    // ROS_INFO("------- PICKING PIPELINE --------");
    // moveGroupInterface.setPlanningTime(4.0);
    // pick(moveGroupInterface, target_pose1.pose);
  }

  void executeAsync()
  {
  }
};
} // namespace cl_movegroup
} // namespace sm_moveit