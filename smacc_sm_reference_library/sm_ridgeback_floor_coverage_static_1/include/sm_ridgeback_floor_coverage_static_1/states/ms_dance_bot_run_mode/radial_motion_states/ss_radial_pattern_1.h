#include <smacc/smacc.h>

namespace sm_ridgeback_floor_coverage_static_1 {
namespace SS1 {
namespace sm_ridgeback_floor_coverage_static_1 {
namespace radial_motion_states {

//FORWARD DECLARATION OF INNER STATES
class StiRadialRotate;
class StiRadialReturn;
class StiRadialEndPoint;
class StiRadialLoopStart;
} // namespace radial_motion_states
} // namespace sm_ridgeback_floor_coverage_static_1
using namespace sm_ridgeback_floor_coverage_static_1::radial_motion_states;

// STATE DECLARATION
struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, MsDanceBotRunMode, StiRadialLoopStart>
{
public: 
        using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvLoopEnd<StiRadialLoopStart>, StNavigateReverse1, ENDLOOP> 

    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
    }

    int iteration_count = 0;
    static constexpr int total_iterations() { return 16; }
    static constexpr float ray_angle_increment_degree() { return 360.0 / total_iterations(); }
    static constexpr float ray_length_meters() { return 5; }
};


// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsRadialPattern1;
#include "sti_radial_end_point.h"
#include "sti_radial_return.h"
#include "sti_radial_rotate.h"
#include "sti_radial_loop_start.h"
} // namespace SS1
} // namespace sm_ridgeback_floor_coverage_static_1