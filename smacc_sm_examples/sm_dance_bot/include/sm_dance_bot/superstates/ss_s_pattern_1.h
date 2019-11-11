// Rotate (90)->Fwd(20)->Rotate (-90)->Fwd (2)->Rotate (-90)->Fwd(20)->Rotate (90)->Fwd(2)

namespace SS5
{

//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class SsrSPatternRotate1;
class SsrSPatternForward1;
class SsrSPatternRotate2;
class SsrSPatternForward2;
class SsrSPatternRotate3;
class SsrSPatternForward3;
class SsrSPatternRotate4;
class SsrSPatternForward4;
class SsrSPatternLoopStart;

enum class TDirection
{
    LEFT,
    RIGHT
};

struct SsSPattern1 : smacc::SmaccState<SsSPattern1, MsDanceBotRunMode, SsrSPatternLoopStart>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        smacc::transition<EvLoopEnd<SsrSPatternLoopStart>, StRotateDegrees6, ENDLOOP>//,

        // Keyboard events
        //smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees6>,
        //smacc::transition<EvKeyPressP<SbKeyboard>, StNavigateToWaypointsX>,

        // Error events
        //smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
        //smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>

        // Internal events
        >
        reactions;

    static void onDefinition()
    {
        //static_configure<KeyboardOrthogonal, SbKeyboard>();
        //static_configure<ObstaclePerceptionOrthogonal, SbLidarSensor>();
    }

    static constexpr float pitch1_lenght_meters() { return 0.6; }
    static constexpr float pitch2_lenght_meters() { return 3.2; }
    static constexpr int total_iterations() { return 3; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    int iteration_count;

    void onInitialize()
    {
        this->iteration_count = 0;
    }
};

//forward declaration for the superstate
using SS = SsSPattern1;

#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_rotate_1.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_forward_1.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_rotate_2.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_forward_2.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_rotate_3.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_forward_3.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_rotate_4.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_forward_4.h>
#include <sm_dance_bot/superstate_routines/s_pattern/ssr_spattern_loop_start.h>

} // namespace SS5
