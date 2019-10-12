
namespace SS3
{
//forward declaration for initial ssr
class SsrRadialRotate;
class SsrRadialReturn;

struct SsRadialPattern3 : smacc::SmaccState<SsRadialPattern3, SmDanceBot, SsrRadialRotate>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
                      // Expected event
                      sc::transition<EvStateFinish<SsRadialPattern3>, StRotateDegrees4>,

                      // Keyboard events
                      sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees4>,
                      sc::transition<EvKeyPressP<SbKeyboard>,StNavigateToWaypointsX>,
                       
                      // Error events
                      sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
                      sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

                      // Internal events
                      sc::custom_reaction<smacc::EvStateFinish<SsrRadialReturn>>
            > reactions;

    int iteration_count;
    int total_iterations;
    float ray_angle_increment_degree;
    float ray_length_meters;

    void onInitialize()
    {
        iteration_count = 0;
        total_iterations = 4;
        ray_angle_increment_degree = 90;
        ray_length_meters = 3;

        this->configure<KeyboardOrthogonal>(std::make_shared<SbKeyboard>());
    }

    sc::result react(const smacc::EvStateFinish<SsrRadialReturn> &ev)
    {
        if (++iteration_count == total_iterations) // 1 == two times
        {
            this->throwFinishEvent();
        }

        return forward_event();
    }
};

//forward declaration for the superstate
using SS = SsRadialPattern3;
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_rotate.h>
} // namespace SS3
