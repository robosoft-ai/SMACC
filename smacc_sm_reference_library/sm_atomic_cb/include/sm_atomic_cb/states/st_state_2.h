#include <smacc/smacc.h>

namespace sm_atomic_cb
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomicCB>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State1, SUCCESS>
    
    >reactions;

    
// STATE FUNCTIONS   
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
    }

    void runtimeConfigure()
    {
        ROS_INFO("Entering State2");

        // get reference to the client
        ClRosTimer *client;
        this->requiresClient(client);

        // subscribe to the timer client callback
        client->onTimerTick(&State2::onTimerClientTickCallback, this);

         // getting reference to the single countdown behavior
        auto *cbsingle = this->getOrthogonal<OrTimer>()
                             ->getClientBehavior<CbTimerCountdownOnce>();

        // subscribe to the single countdown behavior callback
        cbsingle->onTimerTick(&State2::onSingleBehaviorTickCallback, this);
    }
       
    // fire callback function
    void onTimerClientTickCallback()
    {
        ROS_INFO("timer client tick!");
    }

    // fire callback function
    void onSingleBehaviorTickCallback()
    {
        ROS_INFO("single behavior tick!");
    }
};
}