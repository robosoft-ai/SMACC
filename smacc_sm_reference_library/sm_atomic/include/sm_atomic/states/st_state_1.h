#include <smacc/smacc.h>

namespace sm_atomic
{
using namespace ros_timer_client;

struct State1
    : smacc::SmaccState<State1, SmAtomicStateMachine>
{
    typedef mpl::list<smacc::transition<EvTimer<CbTimerRepeatCountdown, OrTimer>, State2>> reactions;

    using SmaccState::SmaccState;

    static void onDefinition()
    {
        static_configure<OrTimer, CbTimerRepeatCountdown>(10);
        static_configure<OrTimer, CbTimerSingleCountdown>(10);
    }

    void onInitialize()
    {
        ClRosTimer *client;
        this->requiresClient(client);

        client->onTimerTick(
            [=]() {
                ROS_INFO("timer tick!");
            });
    }
};
} // namespace sm_atomic