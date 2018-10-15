#pragma once
#include "radial_motion.h"
#include <ros/ros.h>

namespace NavigateToEndPoint
{
struct NavSubstates;
struct ReelSubstates;

//--------------------------------------------
/// State State
struct State : SmaccState< State, RadialMotionStateMachine,
                               mpl::list< NavSubstates, ReelSubstates > >
{
    typedef sc::transition< EvStateFinished, ReturnToRadialStart::State> reactions;

    public:
    State(my_context ctx)
      :SmaccState<State, RadialMotionStateMachine, mpl::list< NavSubstates, ReelSubstates > >(ctx)
    {
          ROS_INFO("Initializating Navigate to endpoint state");
    }
    
    ~State()
    {

    }
};

//------------------------------------------------------------------------------
// orthogonal line 0

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
      typedef mpl::list<sc::custom_reaction< EvActionClientSuccess >> reactions;

      public:

      MoveBaseSubState(my_context ctx) : SmaccState<MoveBaseSubState, NavSubstates >(ctx)
      {
          ROS_INFO("Entering MoveBaseSubState");
          moveBaseClient_ = context<RadialMotionStateMachine >().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");   
      
          smacc::SmaccMoveBaseActionClient::Goal goal;

          geometry_msgs::PoseStamped radialStartPose;
          context<RadialMotionStateMachine >().getData("radial_start_pose", radialStartPose);

          goal.target_pose= radialStartPose;
          goal.target_pose.header.stamp=ros::Time::now();

          int i;
          context<RadialMotionStateMachine >().getData("angle_index", i);
          
          double yaw = i * 10* 180.0/M_PI;

          double dist = 4.5;
          goal.target_pose.pose.position.x+= cos(yaw)*dist;
          goal.target_pose.pose.position.y+= sin(yaw)*dist;
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

//------------------------------------------------------------------------------
// orthogonal line 1
struct ReelSubstates: SmaccState<ReelSubstates, State::orthogonal< 1 > >
{
      public:
      ReelSubstates(my_context ctx)
        :SmaccState<ReelSubstates, State::orthogonal< 1 > >(ctx)
      {

      }
};
}