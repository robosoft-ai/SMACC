#pragma once

#include "smacc/smacc_state_machine_base.h"


using namespace smacc;

struct EvStateFinished : sc::event< EvStateFinished > 
{
};

// ------------------------------------------------------------------------
namespace NavigateToRadialStart
{
    struct State;
};

struct RotateDegress;

struct RadialMotionStateMachine: public SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::State>
{
    RadialMotionStateMachine( my_context ctx, SignalDetector* signalDetector)
        :SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::State>(ctx,signalDetector)
    {

    }
};
