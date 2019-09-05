using namespace smacc;

namespace RadialMotion3
{
    using RadialMotionStateMachine = RadialMotion3;   

    struct RadialMotion3: SmaccState<RadialMotion3,RadialMotionWaypointsStateMachine, RadialMotionSuperState>
    {
        public:
        using SmaccState::SmaccState;
        typedef sc::transition<EvStateFinish<RadialMotion3>, RadialMotion1::RadialMotion1> reactions;

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
