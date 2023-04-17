#include <smacc/smacc.h>

namespace sm_atomic_mode_states
{
using namespace cl_ros_timer;
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct MsState1 : smacc::SmaccState<MsState1, SmAtomic, State1>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbUpdatableTest>();
    }

    void runtimeConfigure()
    {
    }

    void onEntry()
    {
        ROS_INFO("On Entry!");
    }

    void onExit()
    {
        ROS_INFO("On Exit!");
    }

};
} // namespace sm_atomic_mode_states
