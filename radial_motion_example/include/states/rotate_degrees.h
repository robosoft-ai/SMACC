#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>

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
  typedef sc::transition<EvStateFinished, NavigateToEndPoint::NavigateToEndPoint> reactions;

public:
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  RotateDegress(my_context ctx)
      : SmaccState<RotateDegress, RadialMotionStateMachine,
                   mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("-------");
    ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~RotateDegress() { ROS_INFO("Exiting in ROTATE TEN DEGREES STATE"); }
};

//------------------------------------------------------------------
// orthogonal line 0Reel_ActionClient
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, RotateDegress::orthogonal<0>,
                 Navigate> 
{
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  NavigationOrthogonalLine(my_context ctx)
      : SmaccState<NavigationOrthogonalLine, RotateDegress::orthogonal<0>,
                   Navigate>(ctx) // call the SmaccState base constructor
    {
    }
};
//------------------------------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the RotateDegreess State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> 
{
  // this state reacts to the following list of events:
  typedef mpl::list<sc::custom_reaction<EvActionResult<smacc::SmaccMoveBaseActionClient::Result> >,
                    sc::custom_reaction<EvReelInitialized>> reactions;

public:

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx) : SmaccState<Navigate, NavigationOrthogonalLine>(ctx) {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    moveBaseClient_ =
        context<RadialMotionStateMachine>()
            .requiresComponent<smacc::SmaccMoveBaseActionClient>(
                "move_base");

    // read parameters from ros parameter server
    readParameters();

    int i;
    if (!context<RadialMotionStateMachine>().getData("angle_index", i)) 
    {
      // this is the first radial motion (influences to the initial angle)
      i = initial_orientation_index_;
    } 
    else 
    {
      i += 1; // this is not the first time, increment the degress with 10 degreess
    }

    // sets from the state machine i "global variable" to know the current orientation
    ROS_INFO("[RotateDegrees/Navigate] Radial angle index: %d", i);
    context<RadialMotionStateMachine>().setData("angle_index", i);

    // get the angle according to the angle index
    yaw = i * angles::from_degrees(angle_increment_degree_);
    context<RadialMotionStateMachine>().setData("current_yaw", yaw);
    ROS_INFO_STREAM("[RotateDegrees/Navigate] current yaw: " << yaw);

    // check if the motion is in the latest straight motion
    if(i == linear_trajectories_count_) 
    {
      ROS_WARN("STOP ROTATING. TERMINAL STATE OF THE STATE MACHINE");
      terminate();
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

  // when the reel substate is finished we will react starting the motion
  sc::result react(const EvReelInitialized &ev) 
  {
      ROS_INFO("Reacting to reel initialized state. Rotating...");
      rotateTenDegrees();
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void rotateTenDegrees() 
  {
    geometry_msgs::PoseStamped radialStart;
    context<RadialMotionStateMachine>().getData("radial_start_pose", radialStart);

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose = radialStart;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    moveBaseClient_->sendGoal(goal);
  }

  // this is the callback when the navigate action of this state is finished
  // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
  sc::result react(const EvActionResult<smacc::SmaccMoveBaseActionClient::Result>  &ev) 
  {
    if (ev.client == moveBaseClient_) 
    {
      if (ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
        ROS_INFO("Received event to movebase: %s",ev.getResult().toString().c_str());
        
        // notify the parent State to finish via event (the current parent state reacts to this event)
        post_event(EvStateFinished());
        
        // declare this substate as finished
        return discard_event();
        //return terminate();
      }
      else 
      {
        // repeat the navigate action request to the move base node if we get ABORT as response
        // It may work if try again. Move base sometime rejects the request because it is busy.
        rotateTenDegrees();

        // this event was for us. We have used it without moving to any other state. Do not let others consume it.
        return discard_event();
      } 
    }
    else 
    {
      // the action client event success is not for this substate. Let others process this event.
      ROS_INFO("navigate substate lets other process the EvActionResultEv");
      return forward_event();
    }
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
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

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
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, RotateDegress::orthogonal<1>, ToolSubstate>(ctx) // call the SmaccState base constructor                 
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

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  ToolSubstate(my_context ctx) 
    : SmaccState<ToolSubstate, ToolOrthogonalLine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering ToolSubstate");
  }
};
}
