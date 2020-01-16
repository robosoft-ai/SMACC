#include <smacc/smacc.h>
namespace sm_dance_bot
{
namespace SS1
{

namespace sm_dance_bot
{
namespace radial_motion_states
{
//forward declaration for initial ssr
class StiRadialRotate;
class StiRadialReturn;
class StiRadialEndPoint;
class StiRadialLoopStart;
} // namespace radial_motion_states
} // namespace sm_dance_bot

using namespace sm_dance_bot::radial_motion_states;

struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, MsDanceBotRunMode, StiRadialLoopStart>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<

        // Expected event
        smacc::Transition<EvLoopEnd<StiRadialLoopStart>, StRotateDegrees1, ENDLOOP>
        // Keyboard event
        //smacc::Transition<EvKeyPressN<CbDefaultKeyboardBehavior>, StRotateDegrees1>,
        //smacc::Transition<EvKeyPressP<CbDefaultKeyboardBehavior>, StNavigateToWaypointsX>,

        // Error events
        //smacc::Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
        //smacc::Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>>
        >

        reactions;

    static constexpr int total_iterations() { return 20; }
    static constexpr float ray_angle_increment_degree() { return 360.0 / total_iterations(); }
    static constexpr float ray_length_meters() { return 4; }

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

#include <sm_dance_bot/states/radial_motion_states/sti_radial_end_point.h>
#include <sm_dance_bot/states/radial_motion_states/sti_radial_return.h>
#include <sm_dance_bot/states/radial_motion_states/sti_radial_rotate.h>
#include <sm_dance_bot/states/radial_motion_states/sti_radial_loop_start.h>
} // namespace SS1
} // namespace sm_dance_bot