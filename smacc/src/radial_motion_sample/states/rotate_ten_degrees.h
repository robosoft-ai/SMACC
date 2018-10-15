#pragma once

#include "radial_motion.h"
#include <tf/tf.h>

//--------------------------------------------
/// State NavigateToRadialStart
namespace RotateDegress
{ 
  struct NavSubstates;
  class MoveBaseSubState;


 struct EvReelInitialized : sc::event< EvReelInitialized >
    {

    };


  struct ReelSubstates;

  struct State : SmaccState< State, RadialMotionStateMachine, mpl::list< NavSubstates, ReelSubstates > >
  {
      typedef sc::transition< EvStateFinished, NavigateToEndPoint::State> reactions;

      public:
        State(my_context ctx)
            :SmaccState< State, RadialMotionStateMachine, mpl::list< NavSubstates, ReelSubstates > >(ctx)
        {
            ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
        }
      
        ~State()
        {
            ROS_INFO("Exiting in ROTATE TEN DEGREES STATE");
        }
  };


//------------------------------------------------------------------
// orthogonal line 0Reel_ActionClient
  struct NavSubstates: SmaccState<NavSubstates, State::orthogonal< 0 > , MoveBaseSubState>
  {
        public:
        NavSubstates(my_context ctx)
            :SmaccState<NavSubstates, State::orthogonal< 0 > , MoveBaseSubState>(ctx)
    {

    }
  };
//---------------------
  struct MoveBaseSubState: SmaccState<MoveBaseSubState, NavSubstates >
  {
      typedef mpl::list<sc::custom_reaction< EvActionClientSuccess >, 
                        sc::custom_reaction< EvReelInitialized> > reactions;

      public:

      MoveBaseSubState(my_context ctx) : SmaccState<MoveBaseSubState, NavSubstates >(ctx)
      {
          ROS_INFO("Entering MoveBaseSubState");
          moveBaseClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");   
      }


      sc::result react( const EvReelInitialized & ev )
      {
          geometry_msgs::PoseStamped radialStart;
          context<RadialMotionStateMachine >().getData("radial_start_pose",radialStart);
      
          smacc::SmaccMoveBaseActionClient::Goal goal;
          goal.target_pose = radialStart;
          goal.target_pose.header.stamp=ros::Time::now();

          int i;
          if (!context<RadialMotionStateMachine >().getData("angle_index", i))
          {
            i = -9;
          }
          else
          {
            i+= 1;  
          }

          context<RadialMotionStateMachine >().setData("angle_index", i);
          ROS_INFO("Radial angle index: %d", i);
          double yaw = i * 10* 180.0/M_PI;

          goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, yaw);

          moveBaseClient_->sendGoal(goal);
      }

      sc::result react( const EvActionClientSuccess & ev )
      {
          ROS_INFO("Received event to movebase: %s", ev.getResult().toString().c_str());

          if (ev.client == moveBaseClient_ && ev.getResult()==actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              ROS_INFO("move base, goal position reached");
              this->post_event(EvStateFinished());
              //return this->terminate();
          }
          else
          {
              return forward_event();
          }
      }

      ~MoveBaseSubState()
      {
          ROS_INFO("Exiting move goal Action Client");
      }

      private:
          smacc::SmaccMoveBaseActionClient* moveBaseClient_;
  };

//------------------------------------------------------------------
// orthogonal line 1
struct ReelSubState;

struct ReelSubstates: SmaccState<ReelSubstates, State::orthogonal< 1 >, ReelSubState >
{
  public:
    ReelSubstates(my_context ctx)
      :SmaccState<ReelSubstates, State::orthogonal< 1 > , ReelSubState>(ctx)
    {
    }
};

 struct ReelSubState: SmaccState<ReelSubState, ReelSubstates >
    {
        typedef sc::custom_reaction< EvActionClientSuccess > reactions;

        public:
            ReelSubState(my_context ctx)
                : SmaccState<ReelSubState, ReelSubstates >(ctx)
            {
                ROS_INFO("Entering ReelSubState");
                reelActionClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccReelActionClient>("non_rt_helper");
                
                //dispense
                smacc::SmaccReelActionClient::Goal goal;
                goal.dispense_mode= smacc::SmaccReelActionClient::Goal::DISPENSE;
                reelActionClient_->sendGoal(goal);
            }

            sc::result react( const EvActionClientSuccess &  ev)
            {
                ROS_INFO("Reel substate: Received event for reel client");

                if (ev.client == reelActionClient_)
                {
                    ROS_INFO("Received event for reel client");
                    this->post_event(EvReelInitialized());
                    return this->terminate();
                }
                else
                {
                    return forward_event();
                }
            }

            ~ReelSubState()
            {
                ROS_INFO("Exiting Reel_Action Client");
            }

        private:
            smacc::SmaccReelActionClient* reelActionClient_;
    };
}

