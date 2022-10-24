#include <smacc/smacc.h>

namespace sm_fetch_two_table_whiskey_pour
{
    namespace SS3
    {
        namespace sm_fetch_two_table_whiskey_pour
        {
            namespace recovery_screw
            {
                //FORWARD DECLARATION OF INNER STATES
                class StRecoveryScrewInitial;
                class StRecoveryScrewSecond;
                class StRecoveryScrewThird;
            } // namespace recovery_screw
        }     // namespace sm_fetch_two_table_whiskey_pour

        using namespace sm_fetch_two_table_whiskey_pour::recovery_screw;

        // STATE DECLARATION
        struct SsRecoveryScrew : smacc::SmaccState<SsRecoveryScrew, SmFetchTwoTableWhiskeyPour, StRecoveryScrewInitial>
        {
        public:
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                //Transition<EvCbSuccess<CbMoveLastTrajectoryInitialState, OrArm>, StPouringPosture, SUCCESS>
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
#include <sm_fetch_two_table_whiskey_pour/states/recovery_screw_states/st_recovery_screw_initial.h>
#include <sm_fetch_two_table_whiskey_pour/states/recovery_screw_states/st_recovery_screw_second.h>
#include <sm_fetch_two_table_whiskey_pour/states/recovery_screw_states/st_recovery_screw_third.h>

    } // namespace SS3
} // namespace sm_fetch_two_table_whiskey_pour
