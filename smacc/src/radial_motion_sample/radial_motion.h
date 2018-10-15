#pragma once

#include "smacc/smacc_state_machine_base.h"


using namespace smacc;

struct EvStateFinished : sc::event< EvStateFinished > 
{
};

// --------------------- STATES ---------------------------------------------------
namespace NavigateToRadialStart
{
    struct State;
};

namespace RotateDegress
{
    struct State;
}

namespace NavigateToEndPoint
{
    struct State;
}

namespace ReturnToRadialStart
{
    struct State;
}

// --------------------- Radial motion State Machine ---------------------------------------------------
struct RadialMotionStateMachine: public SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::State>
{
    RadialMotionStateMachine( my_context ctx, SignalDetector* signalDetector)
        :SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::State>(ctx,signalDetector)
    {

    }
};
