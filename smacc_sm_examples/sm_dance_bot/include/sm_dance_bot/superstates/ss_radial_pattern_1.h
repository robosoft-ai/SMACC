
namespace SS1
{
//forward declaration for initial ssr
class SsrRadialRotate;
class SsrRadialReturn;
class SsrRadialEndPoint;
class SsrRadialLoopStart;

struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, MsDanceBotRunMode, SsrRadialLoopStart>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<

        // Expected event
        smacc::transition<EvLoopEnd<SsrRadialLoopStart>, StRotateDegrees1, ENDLOOP>
        // Keyboard event
        //smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees1>,
        //smacc::transition<EvKeyPressP<SbKeyboard>, StNavigateToWaypointsX>,

        // Error events
        //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
        //smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>>
        >

        reactions;

    static constexpr int total_iterations() { return 2; }
    static constexpr float ray_angle_increment_degree() { return 90; }
    static constexpr float ray_length_meters() { return 3; }

    int iteration_count = 0;

    static void onDefinition()
    {
        //static_configure<KeyboardOrthogonal, SbKeyboard>();
        //static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
    }

    void onInitialize()
    {
    }
};
//forward declaration for the superstate
using SS = SsRadialPattern1;

#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_rotate.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_loop_start.h>
} // namespace SS1