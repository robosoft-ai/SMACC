#include <smacc/smacc.h>

namespace sm_dance_bot_2
{
namespace SS1
{

namespace sm_dance_bot_2
{
namespace radial_motion_states
{
//forward declaration for initial ssr
class SsrRadialRotate;
class SsrRadialReturn;
class SsrRadialEndPoint;
class SsrRadialLoopStart;
} // namespace radial_motion_states
} // namespace sm_dance_bot

using namespace sm_dance_bot_2::radial_motion_states;

struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, SmDanceBot2, SsrRadialLoopStart>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<

        // Expected event
        //smacc::Transition<EvLoopEnd<SsrRadialLoopStart>, StRotateDegrees1, ENDLOOP>
        // Keyboard event
        //smacc::Transition<EvKeyPressN<CbDefaultKeyboardBehavior>, StRotateDegrees1>,
        //smacc::Transition<EvKeyPressP<CbDefaultKeyboardBehavior>, StNavigateToWaypointsX>,

        // Error events
        //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
        //smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
        >

        reactions;

    static constexpr int total_iterations() { return 20; }
    static constexpr float ray_angle_increment_degree() { return 360.0 / (float)total_iterations(); }

    int iteration_count = 0;

    static void onDefinition()
    {
        //static_configure<OrObstaclePerception, CbLidarSensor>();
    }

    void onInitialize()
    {
    }
};
//forward declaration for the superstate
using SS = SsRadialPattern1;

#include <sm_dance_bot_2/states/radial_motion_states/ssr_radial_end_point.h>
#include <sm_dance_bot_2/states/radial_motion_states/ssr_radial_return.h>
#include <sm_dance_bot_2/states/radial_motion_states/ssr_radial_rotate.h>
#include <sm_dance_bot_2/states/radial_motion_states/ssr_radial_loop_start.h>
} // namespace SS1
} // namespace sm_dance_bot