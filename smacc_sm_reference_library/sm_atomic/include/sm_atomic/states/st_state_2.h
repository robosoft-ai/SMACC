#include <smacc/smacc.h>

namespace sm_atomic
{
struct State2
    : smacc::SmaccState<State2, SmAtomic>
{
    using SmaccState::SmaccState;
    void onInitialize()
    {
        ROS_INFO("Entering State2");
    }
};
}