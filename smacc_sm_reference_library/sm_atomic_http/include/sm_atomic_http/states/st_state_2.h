#include <smacc/smacc.h>

namespace sm_atomic_http
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomicHttp>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvHttp<CbRequest, OrHttp>, State1, SUCCESS>
    
    >reactions;

    
// STATE FUNCTIONS   
    static void staticConfigure()
    {
        configure_orthogonal<OrHttp, CbRequest>(); // EvTimer triggers once at 10 client ticks
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