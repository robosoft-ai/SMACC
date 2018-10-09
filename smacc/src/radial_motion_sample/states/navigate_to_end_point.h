#pragma once
#include "radial_motion.h"
#include <ros/ros.h>

struct NavigateToEndPoint_NavSubstates;
struct NavigateToEndPoint_ReelSubstates;

//--------------------------------------------
/// State NavigateToEndPoint
struct NavigateToEndPoint : SmaccState< NavigateToEndPoint, RadialMotionStateMachine,
                               mpl::list< NavigateToEndPoint_NavSubstates, NavigateToEndPoint_ReelSubstates > >
{
    public:
    NavigateToEndPoint(my_context ctx)
    :SmaccState<NavigateToEndPoint, RadialMotionStateMachine,
                               mpl::list< NavigateToEndPoint_NavSubstates, NavigateToEndPoint_ReelSubstates > >(ctx)
    {
          ROS_INFO("Initializating Navigate to endpoint state");
    }
    
    ~NavigateToEndPoint()
    {

    }
};

//------------------------------------------------------------------------------
// orthogonal line 0

struct MoveBase_MoveGoal_ActionClient;

struct NavigateToEndPoint_NavSubstates: sc::simple_state<
  NavigateToEndPoint_NavSubstates, NavigateToEndPoint::orthogonal< 0 >  , MoveBase_MoveGoal_ActionClient>
{
  NavigateToEndPoint_NavSubstates()
  {
    ROS_INFO("Entering in move_base orthogonal line");
  }
  ~NavigateToEndPoint_NavSubstates()
  {
    ROS_INFO("Exiting in move_base orthogonal line"); 
  }
};

struct MoveBase_MoveGoal_ActionClient: sc::simple_state<
  MoveBase_MoveGoal_ActionClient, NavigateToEndPoint_NavSubstates >
{
    MoveBase_MoveGoal_ActionClient()
    {
      ROS_INFO("Entering MoveBase_MoveGoal_ActionClient");
    }

    ~MoveBase_MoveGoal_ActionClient()
    {
      ROS_INFO("Exiting move goal Action Client");
    }
};


//------------------------------------------------------------------------------
// orthogonal line 1
struct NavigateToEndPoint_ReelSubstates: sc::simple_state<
  NavigateToEndPoint_ReelSubstates, NavigateToEndPoint::orthogonal< 1 > >
{

};

