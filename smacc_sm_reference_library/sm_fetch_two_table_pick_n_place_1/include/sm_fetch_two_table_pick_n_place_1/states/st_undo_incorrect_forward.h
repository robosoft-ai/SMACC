#include <smacc/smacc.h>
namespace sm_fetch_two_table_pick_n_place_1
{
    // STATE DECLARATION
    struct StUndoIncorrectForward : smacc::SmaccState<StUndoIncorrectForward, SmFetchTwoTablePickNPlace1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StForwardNextTable, SUCCESS>>
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
        }

        void runtimeConfigure()
        {
        }
    };
} // namespace sm_fetch_two_table_pick_n_place_1
