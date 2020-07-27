#include <smacc/smacc.h>

namespace sm_ridgeback_floor_coverage_static_1 {
namespace f_pattern_states {

enum class TDirection
{
    LEFT,
    RIGHT
};

// FORWARD DECLARATIONS OF INNER STATES
template <typename SS> class StiFPatternRotate1;
template <typename SS> class StiFPatternForward1;
template <typename SS> class StiFPatternReturn1;
template <typename SS> class StiFPatternRotate2;
template <typename SS> class StiFPatternForward2;
template <typename SS> class StiFPatternStartLoop;
} // namespace f_pattern_states
} // namespace sm_ridgeback_floor_coverage_static_1
namespace sm_ridgeback_floor_coverage_static_1 {
namespace SS4 {
using namespace f_pattern_states;

// STATE DECLARATION
struct SsFPattern1 : smacc::SmaccState<SsFPattern1, MsDanceBotRunMode, StiFPatternStartLoop<SsFPattern1>>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
        
    Transition<EvLoopEnd<StiFPatternStartLoop<SsFPattern1>>, StNavigateForward2, ENDLOOP> //,
        
    >reactions;

// STATE VARIABLES
    // superstate parameters
    static constexpr float ray_lenght_meters() { return 3.25; }
    static constexpr float pitch_lenght_meters() { return 0.75; }
    static constexpr int total_iterations() { return 10; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    // superstate state variables
    int iteration_count;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
        iteration_count = 0;
    }
}; // namespace SS4

// FORWARD DECLARATION FOR THE SUPERSTATE

} // namespace SS4
} // namespace sm_ridgeback_floor_coverage_static_1

#include "sti_fpattern_rotate_1.h"
#include "sti_fpattern_forward_1.h"
#include "sti_fpattern_return_1.h"
#include "sti_fpattern_rotate_2.h"
#include "sti_fpattern_forward_2.h"
#include "sti_fpattern_loop_start.h"
