#include <smacc/smacc.h>

namespace sm_dance_bot_2 {
namespace SS1 {
namespace sm_dance_bot_2 {
namespace radial_motion_states {

//FORWARD DECLARATION OF INNER STATES
class StiRadialRotate;
class StiRadialReturn;
class StiRadialEndPoint;
class StiRadialLoopStart;
} // namespace radial_motion_states
} // namespace sm_dance_bot_2
using namespace sm_dance_bot_2::radial_motion_states;

// STATE DECLARATION
struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, SmDanceBot2, StiRadialLoopStart>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiRadialLoopStart>, StNavigateToWaypointsX, ENDLOOP>
    
    >reactions;

// STATE VARIABLES
    int total_iterations()
    {
        int rays = 5; // default value
        this->getParam("rays_count", rays);
        
        return rays;
    }
    
    float ray_angle_increment_degree() 
    { 
        return 360.0 / (float)total_iterations(); 
    }

    int iteration_count = 0;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
    }
};
//forward declaration for the superstate
using SS = SsRadialPattern1;

#include <sm_dance_bot_2/states/radial_motion_states/sti_radial_end_point.h>
#include <sm_dance_bot_2/states/radial_motion_states/sti_radial_return.h>
#include <sm_dance_bot_2/states/radial_motion_states/sti_radial_rotate.h>
#include <sm_dance_bot_2/states/radial_motion_states/sti_radial_loop_start.h>
} // namespace SS1
} // namespace sm_dance_bot_2