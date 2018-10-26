#pragma once

#include "radial_motion.h"
#include <angles/angles.h>
#include <tf/tf.h>

namespace ReturnToRadialStart
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ReelOrthogonalLine;
struct Navigate;
struct ReelStartAndDispense;

//--------------------------------------------
/// ReturnToRadialStart State
struct ReturnToRadialStart : SmaccState< ReturnToRadialStart, RadialMotionStateMachine,
                               mpl::list< NavigationOrthogonalLine, ReelOrthogonalLine > > // <- these are the orthogonal lines of this State
{
    // when this state is finished then move to the RotateDegress state
    typedef sc::transition< EvStateFinished, RotateDegress::RotateDegress> reactions;

public:
    // This is the state constructor. This code will be executed when the
    // workflow enters in this substate (that is according to statechart the moment when this object is created)
    // after this, its orthogonal lines are created (see orthogonal line classes).
    ReturnToRadialStart(my_context ctx)
      :SmaccState<ReturnToRadialStart, RadialMotionStateMachine, mpl::list< NavigationOrthogonalLine, ReelOrthogonalLine > >(ctx) // call the SmaccState base constructor
    {
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
      typedef mpl::list<sc::custom_reaction< EvActionClientSuccess >, 
                        sc::custom_reaction< EvReelInitialized >> reactions;

      public:

      double yaw;
      
      // This is the substate constructor. This code will be executed when the
      // workflow enters in this substate (that is according to statechart the moment when this object is created)
      Navigate(my_context ctx) 
        : SmaccState<Navigate, NavigationOrthogonalLine >(ctx) // call the SmaccState base constructor
      {
          ROS_INFO("Entering Navigation");

          // this substate will need access to the "MoveBase" resource or plugin. In this line
          // you get the reference to this resource.
          moveBaseClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");   
      }

      // when the reel substate is finished we will react starting the motion
      sc::result react( const EvReelInitialized & ev )
      {
          int i;
          context<RadialMotionStateMachine >().getData("angle_index", i);
          
          yaw = i * angles::from_degrees(10);

          returnToRadialStart();
      }

      // auxiliar function that defines the motion that is requested to the move_base action server
      void returnToRadialStart()
      {
          smacc::SmaccMoveBaseActionClient::Goal goal;
          geometry_msgs::PoseStamped radialStartPose;

          context<RadialMotionStateMachine >().getData("radial_start_pose", radialStartPose);

          goal.target_pose=radialStartPose;
          goal.target_pose.header.stamp=ros::Time::now();

          goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, yaw);
          moveBaseClient_->sendGoal(goal);
      }

      // this is the callback when the navigate action of this state is finished
      // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
      sc::result react( const EvActionClientSuccess & ev )
      {
          ROS_INFO("Received event to movebase: %s", ev.getResult().toString().c_str());

          int i;
          context<RadialMotionStateMachine >().getData("angle_index", i);

          if (i > 8)
          {
              ROS_WARN("STATE MACHINE END");
              //exit(0);
          } 

          if (ev.client == moveBaseClient_)
          {
              if(ev.getResult()==actionlib::SimpleClientGoalState::SUCCEEDED)
              {
                ROS_INFO("move base, goal position reached");
                post_event(EvStateFinished());
                return forward_event();
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
  };

//------------------------------------------------------------------
// orthogonal line 1


struct ReelOrthogonalLine: SmaccState<ReelOrthogonalLine, ReturnToRadialStart::orthogonal< 1 >, ReelStartAndDispense >
{
  public:
    // This is the orthogonal line constructor. This code will be executed when the
    // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
    ReelOrthogonalLine(my_context ctx)
      :SmaccState<ReelOrthogonalLine, ReturnToRadialStart::orthogonal< 1 > , ReelStartAndDispense>(ctx) // call the SmaccState base constructor
    {
    }
};

//--------------------------------------------
// this is the reel substate inside the reel orthogonal line of the ReturnToRadialStart State
 struct ReelStartAndDispense: SmaccState<ReelStartAndDispense, ReelOrthogonalLine >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        public:
            // This is the substate constructor. This code will be executed when the
            // workflow enters in this substate (that is according to statechart the moment when this object is created)
            ReelStartAndDispense(my_context ctx)
                : SmaccState<ReelStartAndDispense, ReelOrthogonalLine >(ctx) // call the SmaccState base constructor
            {
                ROS_INFO("Entering ReelStartAndDispense");
                // this substate will need access to the "Reel" resource or plugin. In this line
                // you get the reference to this resource.
                reelActionClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccReelActionClient>("non_rt_helper");
                
                retract();
            }

            void retract()
            {
                // use the reel resource to request retract (request to the non_rt_helper)
                smacc::SmaccReelActionClient::Goal goal;
                goal.dispense_mode= smacc::SmaccReelActionClient::Goal::RETRACT;
                reelActionClient_->sendGoal(goal);
            }

            sc::result react( const EvActionClientSuccess &  ev)
            {
                // if the reel request is finished and success, then notify the event to the move base substate
                // and finish this substate
                if (ev.client == reelActionClient_ && ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Received event for reel client");
                    // notify the navigate substate that we finished
                    post_event(EvReelInitialized());

                    // declare the reel substate as finished
                    return terminate();
                }
                else if (ev.client == reelActionClient_ && ev.getResult() == actionlib::SimpleClientGoalState::ABORTED)
                {
                    // try again
                    ROS_INFO("Retry reel retract");
                    retract();

                    return discard_event();
                }
                else
                {
                    // the action client event success is not for this substate. Let others process this event.
                    ROS_INFO("navigate substate lets other process the EvActionClientSuccessEv");
                    return forward_event();
                }
            }

            // This is the substate destructor. This code will be executed when the
            // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
            ~ReelStartAndDispense()
            {
                ROS_INFO("Exiting Reel_Action Client");
            }

        private:
            // keeps the reference to the reel resorce or plugin (to connect to the non_rt_helper)
            smacc::SmaccReelActionClient* reelActionClient_;
    };
}
