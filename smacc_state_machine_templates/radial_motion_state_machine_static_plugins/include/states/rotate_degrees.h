/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <thread>

//--------------------------------------------
namespace RotateDegress 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;
struct Navigate;
struct ToolSubstate;

/// State NavigateToRadialStart
struct RotateDegress
    : SmaccState<RotateDegress, RadialMotionStateMachine, mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>> // <- these are the orthogonal lines of this State 
{
  // when this state is finished then move to the NavigateToEndPoint state
  typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>,  NavigateToEndPoint::NavigateToEndPoint> reactions; 

public:

  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("-------");
    ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  void onExit()
  { 
    ROS_INFO("Exiting in ROTATE TEN DEGREES STATE"); 
  }
};

//------------------------------------------------------------------
// orthogonal line 0Reel_ActionClient
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, RotateDegress::orthogonal<0>,
                 Navigate> 
{
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
  }
};
//------------------------------------------------------------------

    // this is the navigate substate inside the navigation orthogonal line of the RotateDegreess State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> 
{
public:

  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource
    this->requiresComponent(moveBaseClient_ , ros::NodeHandle("move_base"));

    this->requiresComponent(odomTracker_ );   

    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    // read parameters from ros parameter server
    readParameters();

    int i;
    if (!this->getGlobalSMData("angle_index", i)) 
    {
      // this is the first radial motion (influences to the initial angle)
      i = initial_orientation_index_;
    } 
    else 
    {
      i += 1; // this is not the first time, increment the degress with 10 degreess
    }

    ROS_INFO("Trajectory %d/%d", i , linear_trajectories_count_);

    // check if the motion is in the latest straight motion
    if(i == linear_trajectories_count_) 
    {
      ROS_WARN("STOP ROTATING. TERMINAL STATE OF THE STATE MACHINE");
      terminate();
    }
    else
    {
      // sets from the state machine i "global variable" to know the current orientation
      ROS_INFO("[RotateDegrees/Navigate] Radial angle index: %d", i);
      this->setGlobalSMData("angle_index", i);

      // get the angle according to the angle index
      yaw = i * angles::from_degrees(angle_increment_degree_);
      this->setGlobalSMData("current_yaw", yaw);
      ROS_INFO_STREAM("[RotateDegrees/Navigate] current yaw: " << yaw);

      this->odomTracker_->clearPath();
      
      ros::spinOnce();
      this->plannerSwitcher_->setForwardPlanner();
      this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::RECORD_PATH_FORWARD);
      
      rotateDegrees();
    }
  }

  // reads parameters from the ros parameter server
  void readParameters()
  {
    this->param("initial_orientation_index", initial_orientation_index_, 0);
    this->param("angle_increment_degree", angle_increment_degree_, 90.0);
    this->param("linear_trajectories_count", linear_trajectories_count_, 4);

    ROS_INFO_STREAM("initial_orientation_index:" << initial_orientation_index_);
    ROS_INFO_STREAM("angle_increment_degree:" << angle_increment_degree_);
    ROS_INFO_STREAM("linear_trajectories_count:" << linear_trajectories_count_);
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void rotateDegrees() 
  {
    ROS_INFO("Rotate degree MoveBase Call");
    geometry_msgs::PoseStamped radialStart;
    this->getGlobalSMData("radial_start_pose", radialStart);

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose = radialStart;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    moveBaseClient_->sendGoal(goal);
  }

  void onExit()
  { 
    ROS_INFO("Exiting move goal Action Client"); 
  }

private:
  // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server). 
  // this resource can be used from any method in this state
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  smacc_odom_tracker::OdomTracker* odomTracker_;

  smacc_planner_switcher::PlannerSwitcher* plannerSwitcher_;

  // the initial index of the linear motion (factor of angle_increment_degrees)
  // value specified in parameters server
  int initial_orientation_index_;

  // the increment of angle between to linear motions
  // value specified in parameters server
  double angle_increment_degree_;

  // value specified in parameters server
  int linear_trajectories_count_;

  // the angle of the current radial motion
  double yaw;
};
//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, RotateDegress::orthogonal<1>, ToolSubstate> {
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  void onExit()
  { 
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};

struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:
    using SmaccState::SmaccState;
    SMACC_STATE_BEHAVIOR(ToolBehaviorKeyName);
};

}
