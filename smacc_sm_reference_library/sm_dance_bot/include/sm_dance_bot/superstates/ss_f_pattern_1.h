
namespace SS4
{

//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class SsrFPatternRotate1;
class SsrFPatternForward1;
class SsrFPatternReturn1;
class SsrFPatternRotate2;
class SsrFPatternForward2;
class SsrFPatternStartLoop;

enum class TDirection
{
    LEFT,
    RIGHT
};

struct SsFPattern1 : smacc::SmaccState<SsFPattern1, MsDanceBotRunMode, SsrFPatternStartLoop>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        smacc::transition<EvLoopEnd<SsrFPatternStartLoop>, StNavigateForward2, ENDLOOP>//,

        // Keyboard events
        //smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees4>,
        //smacc::transition<EvKeyPressP<SbKeyboard>, StNavigateToWaypointsX>,

        // Error events
        //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
        //smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>
        >
        reactions;

    // superstate parameters
    static constexpr float ray_lenght_meters() { return 2; }
    static constexpr float pitch_lenght_meters() { return 0.6; }
    static constexpr int total_iterations() { return 2; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    // superstate state variables
    int iteration_count;

    static void onDefinition()
    {
        //static_configure<KeyboardOrthogonal, SbKeyboard>();
        //static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
    }

    void onInitialize()
    {
        iteration_count = 0;
    }
}; // namespace SS4

//forward declaration for the superstate
using SS = SsFPattern1;
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_rotate_1.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_forward_1.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_return_1.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_rotate_2.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_forward_2.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_loop_start.h>
} // namespace SS4
