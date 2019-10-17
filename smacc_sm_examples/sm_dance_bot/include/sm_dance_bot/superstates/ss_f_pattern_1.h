
namespace SS4
{

//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class SsrFPatternRotate1;
class SsrFPatternForward1;
class SsrFPatternReturn1;
class SsrFPatternRotate2;
class SsrFPatternForward2;

enum class TDirection
{
    LEFT,
    RIGHT
};

struct SsFPattern1 : smacc::SmaccState<SsFPattern1, SmDanceBot, SsrFPatternRotate1>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        smacc::transition<EvStateFinish<SsFPattern1>, StNavigateForward2>,

        // Keyboard events
        smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees4>,
        smacc::transition<EvKeyPressP<SbKeyboard>, StNavigateToWaypointsX>,

        // Error events
        smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
        smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

        // Internal events
        sc::custom_reaction<smacc::EvStateFinish<SsrFPatternRotate2>>>
        reactions;

    static constexpr float ray_lenght_meters() { return 2; }
    static constexpr float pitch_lenght_meters() { return 0.6; }
    static constexpr float total_iterations() { return 2; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    int iteration_count;

    static void onDefinition()
    {
        static_configure<KeyboardOrthogonal, SbKeyboard>();
    }

    void onInitialize()
    {
        iteration_count = 0;
    }

    sc::result react(const smacc::EvStateFinish<SsrFPatternRotate2> &ev)
    {
        ROS_INFO("FPATTERN iteration: %d", iteration_count);
        if (++iteration_count == total_iterations()) // 1 == two times
        {
            this->throwFinishEvent();
        }

        return forward_event();
    }
}; // namespace SS4

//forward declaration for the superstate
using SS = SsFPattern1;
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_rotate_1.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_forward_1.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_return_1.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_rotate_2.h>
#include <sm_dance_bot/superstate_routines/f_pattern/ssr_fpattern_forward_2.h>
} // namespace SS4
