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

enum class TDirection {LEFT, RIGHT } ;

struct SsSPattern1 : smacc::SmaccState<SsSPattern1, SmDanceBot, SsrSPatternRotate1>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
                      // Expected event
                      sc::transition<EvStateFinish<SsSPattern1>, StRotateDegrees6>,

                      // Keyboard events
                      sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees6>,
                      sc::transition<EvKeyPressP<SbKeyboard>,StNavigateToWaypointsX>,
                       
                      // Error events
                      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
                      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient::Result>, StNavigateToWaypointsX>,

                      // Internal events
                      sc::custom_reaction<smacc::EvStateFinish<SsrSPatternForward4>>
            > reactions;


    float pitch1_lenght_meters;
    float pitch2_lenght_meters;
    int iteration_count;
    int total_iterations;

    TDirection direction;

    void onInitialize()
    {
        this->pitch1_lenght_meters = 0.6;
        this->pitch2_lenght_meters = 3.2;
        this->iteration_count = 0 ;
        this->total_iterations = 3;
        this->direction = TDirection::RIGHT;

        this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
    }

    sc::result react(const smacc::EvStateFinish<SsrSPatternForward4> &ev)
    {
        if (++iteration_count == total_iterations) // 1 == two times
        {
            this->throwFinishEvent();
        }

        return forward_event();
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

} // namespace SS3
