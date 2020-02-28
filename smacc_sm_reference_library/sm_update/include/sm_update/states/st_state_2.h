#include <smacc/smacc.h>

namespace sm_update
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmUpdate>, ISmaccUpdatable
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State1, SUCCESS>

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
    }

    void runtimeConfigure()
    {
        ROS_INFO("Entering State2");

        this->setUpdatePeriod(ros::Duration(1));

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

    virtual void update() override
    {
        ROS_INFO("STATE 2 UPDATE");
    }

    void onTimerClientTickCallback()
    {
        ROS_INFO("timer client tick!");
    }

    void onSingleBehaviorTickCallback()
    {
        ROS_INFO("single behavior tick!");
    }
};
}