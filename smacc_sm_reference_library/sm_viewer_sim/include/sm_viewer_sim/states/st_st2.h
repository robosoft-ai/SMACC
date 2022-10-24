#include <smacc/smacc.h>

namespace sm_viewer_sim
{
struct St2 : smacc::SmaccState<St2, MsRunMode>
{
    static int counter;
    typedef mpl::list<
        Transition<Ev2, St3, smacc::SUCCESS>,
        Transition<EvFail, MsRecoveryMode, smacc::ABORT>>
        reactions;

    using SmaccState::SmaccState;

    void onEntry()
    {
        ROS_INFO("St2");

        ROS_INFO_STREAM("counter: " << counter);
        if (counter % 2 == 0)
            delayedPostEvent<EvFail>(this, 3);
        else
            delayedPostEvent<Ev2>(this, 3);

        counter++;
        this->setGlobalSMData("St2Attempts", counter);
    }
};

int St2::counter = 0;

}
