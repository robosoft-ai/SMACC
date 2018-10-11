#pragma once

#include "radial_motion.h"

struct RotateDegrees_NavSubstates;
struct RotateDegress_ReelSubstates;

//--------------------------------------------
/// State NavigateToRadialStart
struct RotateDegress : SmaccState< RotateDegress, RadialMotionStateMachine,
                               mpl::list< RotateDegrees_NavSubstates, RotateDegress_ReelSubstates > >
{
    public:
    RotateDegress(my_context ctx)
    :SmaccState<RotateDegress, RadialMotionStateMachine,
                               mpl::list< RotateDegrees_NavSubstates, RotateDegress_ReelSubstates > >(ctx)
    {
        ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
    }
    
    ~RotateDegress()
    {

    }
};

// orthogonal line 0
struct RotateDegrees_NavSubstates: sc::simple_state<
  RotateDegrees_NavSubstates, RotateDegress::orthogonal< 0 > >
{

};

// orthogonal line 1
struct RotateDegress_ReelSubstates: sc::simple_state<
  RotateDegress_ReelSubstates, RotateDegress::orthogonal< 1 > >
{

};

