#pragma once

#include "radial_motion.h"

struct ReturnToRadialStart_NavSubstates;
struct ReturnToRadialStart_ReelSubstates;

//--------------------------------------------
/// State ReturnToRadialStart
struct ReturnToRadialStart : SmaccState< ReturnToRadialStart, RadialMotionStateMachine,
                               mpl::list< ReturnToRadialStart_NavSubstates, ReturnToRadialStart_ReelSubstates > >
{
    public:
    ReturnToRadialStart(my_context ctx)
    :SmaccState<ReturnToRadialStart, RadialMotionStateMachine,
                               mpl::list< ReturnToRadialStart_NavSubstates, ReturnToRadialStart_ReelSubstates > >(ctx)
    {

    }
    
    ~ReturnToRadialStart()
    {

    }
};

// orthogonal line 0
struct ReturnToRadialStart_NavSubstates: sc::simple_state<
  ReturnToRadialStart_NavSubstates, ReturnToRadialStart::orthogonal< 0 > >
{

};

// orthogonal line 1
struct ReturnToRadialStart_ReelSubstates: sc::simple_state<
  ReturnToRadialStart_ReelSubstates, ReturnToRadialStart::orthogonal< 1 > >
{

};

