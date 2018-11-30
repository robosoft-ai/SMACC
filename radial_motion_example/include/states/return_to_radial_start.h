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
    typedef sc::transition< EvStateFinished, RotateDegress::RotateDegress> reactions;

public:
    // This is the state constructor. This code will be executed when the
    // workflow enters in this substate (that is according to statechart the moment when this object is created)
    // after this, its orthogonal lines are created (see orthogonal line classes).
    ReturnToRadialStart(my_context ctx)
      :SmaccState<ReturnToRadialStart, RadialMotionStateMachine, mpl::list< NavigationOrthogonalLine , ToolOrthogonalLine> >(ctx) // call the SmaccState base constructor
    {
        ROS_INFO("-------");
        ROS_INFO("Entering State: ReturnToRadialStart");
    }
    
    // This is the state destructor. This code will be executed when the
    // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
    ~ReturnToRadialStart()
    {
        ROS_INFO("Exiting State: ReturnToRadialStart");
    }
};

//--------------------------------------------
struct NavigationOrthogonalLine: SmaccState<NavigationOrthogonalLine, ReturnToRadialStart::orthogonal< 0 > , Navigate>
{
public:
    // This is the orthogonal line constructor. This code will be executed when the
    // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
    NavigationOrthogonalLine(my_context ctx)
            :SmaccState<NavigationOrthogonalLine, ReturnToRadialStart::orthogonal< 0 > , Navigate>(ctx) // call the SmaccState base constructor
    {
    }
};

//--------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the ReturnToRadialStart State
struct Navigate: SmaccState<Navigate, NavigationOrthogonalLine >
{
// this state reacts to the following list of events:
typedef mpl::list<
                sc::custom_reaction< EvActionResult<smacc::SmaccMoveBaseActionClient::Result> >, 
                sc::custom_reaction< EvReelInitialized >> reactions;

public:
    // the angle of the current radial motion
    double yaw;
    
    // This is the substate constructor. This code will be executed when the
    // workflow enters in this substate (that is according to statechart the moment when this object is created)
    Navigate(my_context ctx) 
    : SmaccState<Navigate, NavigationOrthogonalLine >(ctx) // call the SmaccState base constructor
    {
        ROS_INFO("Entering Navigation");

        // this substate will need access to the "MoveBase" resource or plugin. In this line
        // you get the reference to this resource.
        context<RadialMotionStateMachine >().requiresComponent<smacc::SmaccMoveBaseActionClient>(moveBaseClient_ , ros::NodeHandle("move_base"));   
    
        // read from the state machine yaw "global variable" to know the current line orientation
        this->getGlobalData("current_yaw", yaw);
        ROS_INFO_STREAM("[ReturnToRadialStart/Navigate] current yaw:" << yaw );
    }

    // when the reel substate is finished we will react starting the motion
    sc::result react( const EvReelInitialized & ev )
    {   
        returnToRadialStart();
    }

    // auxiliar function that defines the motion that is requested to the move_base action server
    void returnToRadialStart()
    {
        smacc::SmaccMoveBaseActionClient::Goal goal;
        geometry_msgs::PoseStamped radialStartPose;

        this->getGlobalData("radial_start_pose", radialStartPose);

        goal.target_pose=radialStartPose;
        goal.target_pose.header.stamp=ros::Time::now();

        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, yaw);
        moveBaseClient_->sendGoal(goal);
    }

    
    // this is the callback when the navigate action of this state is finished
    // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
    sc::result react( const EvActionResult<smacc::SmaccMoveBaseActionClient::Result> & ev )
    {
        ROS_INFO("Received event to movebase: %s", ev.getResult().toString().c_str());

        if (ev.client == moveBaseClient_)
        {
            if(ev.getResult()==actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("move base, goal position reached");
        
                // notify the parent State to finish via event (the current parent state reacts to this event)
                post_event(EvStateFinished());
                
                // declare this substate as finished
                return discard_event();
                //return terminate();
            }
            else if (ev.getResult()==actionlib::SimpleClientGoalState::ABORTED)
            {
                // repeat the navigate action request to the move base node if we get ABORT as response
                // It may work if try again. Move base sometime rejects the request because it is busy.
                returnToRadialStart();

                // this event was for us. We have used it without moving to any other state. Do not let others consume it.
                return discard_event();
            }
        }
        else
        {
            // the action client event success is not for this substate. Let others process this event.
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
    smacc::SmaccMoveBaseActionClient* moveBaseClient_;


    smacc_odom_tracker::OdomTracker* odomTracker_;
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, ReturnToRadialStart::orthogonal<1>, ToolSubstate> {
public:
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, ReturnToRadialStart::orthogonal<1>, ToolSubstate>(ctx) // call the SmaccState base constructor                 
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
    this->requiresComponent(toolActionClient_, ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
}