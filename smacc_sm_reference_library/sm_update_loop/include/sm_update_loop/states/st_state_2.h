#include <smacc/smacc.h>

namespace sm_update_loop
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmUpdateLoop>, ISmaccUpdatable
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

        this->setUpdatePeriod(ros::Duration(1));
    }

    virtual void update() override
    {
        ROS_INFO("STATE 2 UPDATE");
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
