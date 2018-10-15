#pragma once

#include "radial_motion.h"

namespace ReturnToRadialStart
{
struct NavSubstates;
struct ReelSubstates;

 struct EvReelInitialized : sc::event< EvReelInitialized >
    {

    };

//--------------------------------------------
/// State State
struct State : SmaccState< State, RadialMotionStateMachine,
                               mpl::list< NavSubstates, ReelSubstates > >
{
    typedef sc::transition< EvStateFinished, RotateDegress::State> reactions;


    public:
    State(my_context ctx)
      :SmaccState<State, RadialMotionStateMachine, mpl::list< NavSubstates, ReelSubstates > >(ctx)
    {
        ROS_INFO("Entering State: ReturnToRadialStart");
    }
    
    ~State()
    {
        ROS_INFO("Exiting State: ReturnToRadialStart");
    }
};

  struct MoveBaseSubState;
  struct NavSubstates: SmaccState<NavSubstates, State::orthogonal< 0 > , MoveBaseSubState>
  {
    public:
      NavSubstates(my_context ctx)
          :SmaccState<NavSubstates, State::orthogonal< 0 > , MoveBaseSubState>(ctx)
      {

      }
  };

  struct MoveBaseSubState: SmaccState<MoveBaseSubState, NavSubstates >
  {
      typedef mpl::list<sc::custom_reaction< EvActionClientSuccess >, 
                        sc::custom_reaction< EvReelInitialized >> reactions;

      public:

      MoveBaseSubState(my_context ctx) : SmaccState<MoveBaseSubState, NavSubstates >(ctx)
      {
          ROS_INFO("Entering MoveBaseSubState");
          moveBaseClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");   
      }

      sc::result react( const EvReelInitialized & ev )
      {
          smacc::SmaccMoveBaseActionClient::Goal goal;
          geometry_msgs::PoseStamped radialStartPose;

          context<RadialMotionStateMachine >().getData("radial_start_pose", radialStartPose);

          goal.target_pose=radialStartPose;
          goal.target_pose.header.stamp=ros::Time::now();

          int i;
          context<RadialMotionStateMachine >().getData("angle_index", i);
          
          double yaw = i * 10* 180.0/M_PI;
          goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, yaw);

          moveBaseClient_->sendGoal(goal);
      }

      sc::result react( const EvActionClientSuccess & ev )
      {
          ROS_INFO("Received event to movebase: %s", ev.getResult().toString().c_str());

          int i;
          context<RadialMotionStateMachine >().getData("angle_index", i);

          if (i > 8)
          {
              ROS_WARN("STATE MACHINE END");
              exit(0);
          } 

          if (ev.client == moveBaseClient_ && ev.getResult()==actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              ROS_INFO("move base, goal position reached");
              this->post_event(EvStateFinished());
              return this->terminate();
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
                goal.dispense_mode= smacc::SmaccReelActionClient::Goal::RETRACT;
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
