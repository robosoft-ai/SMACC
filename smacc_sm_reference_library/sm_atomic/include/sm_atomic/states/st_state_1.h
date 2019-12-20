#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_action_client.h>

namespace sm_atomic
{
struct State1
    : smacc::SmaccState<State1, SmAtomicStateMachine>
{
    typedef mpl::list<smacc::transition<smacc::EvActionSucceeded<smacc::ClMoveBaseZ, OrNavigation>, State2>> reactions;

    using SmaccState::SmaccState;

    static void onDefinition()
    {
        static_configure<OrNavigation, CbState1>();
    }

    void onInitialize()
    {
    }
};
}