#include <smacc/smacc.h>
namespace sm_fetch_two_table_pick_n_place_1
{
    // STATE DECLARATION
    struct StInitialForward : smacc::SmaccState<StInitialForward, SmFetchTwoTablePickNPlace1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StInitialForward, ABORT>
            
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
            configure_orthogonal<OrNavigation, CbNavigateForward>(1.2);
        }

        void runtimeConfigure()
        {
            ROS_INFO("runtime");
        }

        void OnEntry()
        {
            ROS_INFO("state on entry");
        }
    };
} // namespace sm_fetch_two_table_pick_n_place_1