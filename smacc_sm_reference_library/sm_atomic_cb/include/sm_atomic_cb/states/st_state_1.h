#include <smacc/smacc.h>

namespace sm_atomic_cb
{
using namespace cl_ros_timer;
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmAtomicCB>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State2, SUCCESS>
    
    >reactions;

    
// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
    }

    void runtimeConfigure()
    {
        // get reference to the client
        ClRosTimer *client;
        this->requiresClient(client);

        // subscribe to the timer client callback
        client->onTimerTick(&State1::onTimerClientTickCallback, this);

        // getting reference to the repeat countdown behavior
        auto *cbrepeat = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownLoop>();

        // subscribe to the repeat countdown behavior callback
        cbrepeat->onTimerTick(&State1::onRepeatBehaviorTickCallback, this);

        // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&State1::onSingleBehaviorTickCallback, this);
    }

    // fire callback function
    void onTimerClientTickCallback()
    {
        ROS_INFO("timer client tick!");
    }

    // fire callback function
    void onRepeatBehaviorTickCallback()
    {
        ROS_INFO("repeat behavior tick!");
    }

    // fire callback function
    void onSingleBehaviorTickCallback()
    {
        ROS_INFO("single behavior tick!");
    }
};
} // namespace sm_atomic_cb