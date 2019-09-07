using namespace smacc;

namespace RadialMotion2
{
    using RadialMotionStateMachine = RadialMotion2;   

    struct RadialMotion2: SmaccState<RadialMotion2,RadialMotionWaypointsStateMachine, RadialMotionSuperState>
    {
        public:
        using SmaccState::SmaccState;
        typedef sc::transition<EvStateFinish<RadialMotion2>, Spinning2> reactions;

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

    #include <sm_radial_motion/states.h>
}
