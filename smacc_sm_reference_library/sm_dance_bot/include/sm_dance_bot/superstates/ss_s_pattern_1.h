#include <smacc/smacc.h>

namespace sm_dance_bot
{
namespace SS5
{

namespace sm_dance_bot
{
namespace s_pattern_states
{
//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class StiSPatternRotate1;
class StiSPatternForward1;
class StiSPatternRotate2;
class StiSPatternForward2;
class StiSPatternRotate3;
class StiSPatternForward3;
class StiSPatternRotate4;
class StiSPatternForward4;
class StiSPatternLoopStart;
} // namespace s_pattern_states
} // namespace sm_dance_bot

enum class TDirection
{
    LEFT,
    RIGHT
};

using namespace sm_dance_bot::s_pattern_states;

struct SsSPattern1 : smacc::SmaccState<SsSPattern1, MsDanceBotRunMode, StiSPatternLoopStart>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
        // Expected event
        Transition<EvLoopEnd<StiSPatternLoopStart>, StRotateDegrees6, ENDLOOP> //,

        // Error events
        //Transition<smacc::EvTopicMessageTimeout<CbLidarSensor>, StAcquireSensors>,
        //Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>

        // Internal events
        >
        reactions;

    static void onDefinition()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    static constexpr float pitch1_lenght_meters() { return 0.75; }
    static constexpr float pitch2_lenght_meters() { return 1.75; }
    static constexpr int total_iterations() { return 11; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    int iteration_count;

    void onInitialize()
    {
        this->iteration_count = 0;
    }
};

//forward declaration for the superstate
using SS = SsSPattern1;

#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_1.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_1.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_2.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_2.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_3.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_3.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_4.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_4.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_loop_start.h>

} // namespace SS5
} // namespace sm_dance_bot