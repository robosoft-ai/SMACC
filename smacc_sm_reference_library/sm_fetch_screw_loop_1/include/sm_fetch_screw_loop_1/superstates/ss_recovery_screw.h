#include <smacc/smacc.h>

namespace sm_fetch_screw_loop_1
{
    namespace SS3
    {
        namespace sm_fetch_screw_loop_1
        {
            namespace recovery_screw
            {

                //FORWARD DECLARATION OF INNER STATES
                class StRecoveryScrewInitial;
                class StRecoveryScrewSecond;
                class StRecoveryScrewThird;
            } // namespace recovery_screw
        }     // namespace sm_fetch_screw_loop_1

        using namespace sm_fetch_screw_loop_1::recovery_screw;

        // STATE DECLARATION
        struct SsRecoveryScrew : smacc::SmaccState<SsRecoveryScrew, SmFetchScrewLoop1, StRecoveryScrewInitial>
        {
        public:
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                //Transition<EvCbSuccess<CbMoveLastTrajectoryInitialState, OrArm>, StSecondPosture, SUCCESS>
                Transition<EvSequenceFinished<SS3::StRecoveryScrewThird>, StInitialPosture, SUCCESS>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
            }

            void runtimeConfigure()
            {

            }

            void onExit()
            {
                ros::Duration(2.0).sleep();
            }
        };

        // FORWARD DECLARATION FOR THE SUPERSTATE
        using SS = SsRecoveryScrew;
#include <sm_fetch_screw_loop_1/states/recovery_screw_states/st_recovery_screw_initial.h>
#include <sm_fetch_screw_loop_1/states/recovery_screw_states/st_recovery_screw_second.h>
#include <sm_fetch_screw_loop_1/states/recovery_screw_states/st_recovery_screw_third.h>

    } // namespace SS3
} // namespace sm_fetch_screw_loop_1
