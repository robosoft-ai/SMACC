using namespace smacc;

namespace RadialMotion1
{   
    using RadialMotionStateMachine = RadialMotion1;   

    struct RadialMotion1: SmaccState<RadialMotion1,RadialMotionWaypointsStateMachine, RadialMotionSuperState>
    {
        public:
        using SmaccState::SmaccState;
        //typedef sc::transition<EvStateFinished<RadialMotion1>, RadialMotion2::RadialMotion2> reactions;
        
        void onInitialize()
        {
        }

        void onEntry()
        {
        }

        void onExit()
        {
        }
    };

    #include <radial_motion/states.h>
}


/*

// ALTERNATIVE 1
using namespace smacc;

namespace RadialMotion1
{   
    class NavigateToRadialStart;
    struct RadialMotionSuperState: SmaccState<RadialMotionSuperState,RadialMotionWaypointsStateMachine, NavigateToRadialStart>
    {
        public:
        using SmaccState::SmaccState;
        typedef sc::transition<EvStateFinished<RadialMotionSuperState>, RadialMotion2::RadialMotionSuperState> reactions;
        
        void onInitialize()
        {
        }

        void onEntry()
        {
        }

        void onExit()
        {
        }
    };

    //using RadialMotionSuperState = RadialMotion1::RadialMotion1;   
    #include <radial_motion/states.h>
}



*/