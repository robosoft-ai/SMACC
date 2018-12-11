#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>

namespace ReturnToRadialStart
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// ReturnToRadialStart State
struct ReturnToRadialStart : SmaccState< ReturnToRadialStart, RadialMotionStateMachine,
                               mpl::list< NavigationOrthogonalLine, ToolOrthogonalLine > > // <- these are the orthogonal lines of this State
{
    // when this state is finished then move to the RotateDegress state
    typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>, RotateDegress::RotateDegress> reactions; 

public:
    using SmaccState::SmaccState;

    void onEntry()
    {
        ROS_INFO("-------");
        ROS_INFO("Entering State: ReturnToRadialStart");
    }
    
    void onExit()
    {
        ROS_INFO("Exiting State: ReturnToRadialStart");
    }
};

//--------------------------------------------
struct NavigationOrthogonalLine: SmaccState<NavigationOrthogonalLine, ReturnToRadialStart::orthogonal< 0 > , Navigate>
{
public:
    using SmaccState::SmaccState;

    void onEntry()
    {
    }
};

//--------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the ReturnToRadialStart State
struct Navigate: SmaccState<Navigate, NavigationOrthogonalLine >
{
public:
    // the angle of the current radial motion
    double yaw;
    
    using SmaccState::SmaccState;

    void onEntry()
    {
        ROS_INFO("Entering Navigation");

        // this substate will need access to the "MoveBase" resource or plugin. In this line
        // you get the reference to this resource.
        this->requiresComponent(moveBaseClient_ , ros::NodeHandle("move_base"));   

        this->requiresComponent(odomTracker_);
    
        this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

        // read from the state machine yaw "global variable" to know the current line orientation
        this->getGlobalSMData("current_yaw", yaw);
        ROS_INFO_STREAM("[ReturnToRadialStart/Navigate] current yaw:" << yaw );

        this->plannerSwitcher_->setBackwardPlanner();
        this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::CLEAR_PATH_BACKWARD);

        returnToRadialStart();
    }

    // auxiliar function that defines the motion that is requested to the move_base action server
    void returnToRadialStart()
    {
        smacc::SmaccMoveBaseActionClient::Goal goal;
        geometry_msgs::PoseStamped radialStartPose;

        this->getGlobalSMData("radial_start_pose", radialStartPose);

        goal.target_pose=radialStartPose;
        goal.target_pose.header.stamp=ros::Time::now();

        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, yaw);
        moveBaseClient_->sendGoal(goal);
    }

    // This is the substate destructor. This code will be executed when the
    // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
    ~Navigate()
    {
        ROS_INFO("Exiting move goal Action Client");
    }

    private:
    // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server). 
    // this resource can be used from any method in this state
    smacc::SmaccMoveBaseActionClient* moveBaseClient_;

    smacc_odom_tracker::OdomTracker* odomTracker_;

    smacc_planner_switcher::PlannerSwitcher* plannerSwitcher_;
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, ReturnToRadialStart::orthogonal<1>, ToolSubstate> {
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  ~ToolOrthogonalLine() 
  { 
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};
//---------------------------------------------------------------------------------------------------------
struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> {
  
public:

  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering ToolSubstate");
    this->requiresComponent(toolActionClient_, ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
}