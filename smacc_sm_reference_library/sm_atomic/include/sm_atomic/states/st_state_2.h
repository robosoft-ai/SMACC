#include <smacc/smacc.h>

namespace sm_atomic
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomic>
{
    using SmaccState::SmaccState;
    
// STATE FUNCTIONS    
    void runtimeConfigure()
    {
        ROS_INFO("Entering State2");
    }
};
}