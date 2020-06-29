#include <smacc/smacc.h>
namespace sm_moveit_2
{
// STATE DECLARATION
struct StUndoIncorrectForward : smacc::SmaccState<StUndoIncorrectForward, SmMoveit2>
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
} // namespace sm_moveit_2