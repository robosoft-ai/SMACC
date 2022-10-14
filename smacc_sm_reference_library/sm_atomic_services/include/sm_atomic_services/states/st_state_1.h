#include <smacc/smacc.h>

namespace sm_atomic_services
{
using namespace cl_ros_timer;
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmAtomicServices>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State2, SUCCESS>

    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
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
} // namespace sm_atomic
