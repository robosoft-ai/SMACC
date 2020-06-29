#include <smacc/smacc.h>
namespace sm_moveit_4
{
// STATE DECLARATION
struct StUndoIncorrectForward : smacc::SmaccState<StUndoIncorrectForward, SmMoveIt44>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StForwardNextTable, SUCCESS>
        >
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
} // namespace sm_moveit_4