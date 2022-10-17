#include <smacc/smacc.h>

namespace sm_atomic_services
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomicServices>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvServiceRequestReceieved<CbServiceServer, OrServices>, State1, SUCCESS>

    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrServices, CbServiceServer>();
    }

    void runtimeConfigure()
    {
        ROS_INFO("Entering State2");
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
}
