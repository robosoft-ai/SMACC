
namespace SS1
{
//forward declaration for initial ssr
class SsrRadialRotate;
class SsrRadialReturn;

struct SsRadialPattern1 : smacc::SmaccState<SsRadialPattern1, SmDanceBot, SsrRadialRotate>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<

        // Expected event
        sc::transition<EvStateFinish<SsRadialPattern1>, StRotateDegrees1>,

        // Keyboard event
        sc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees1>,
        sc::transition<EvKeyPressP<SbKeyboard>, StNavigateToWaypointsX>,

        // Error events
        sc::transition<smacc::EvTopicMessageTimeout<LidarSensor>, StAcquireSensors>,
        sc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>,

        // Internal events
        sc::custom_reaction<smacc::EvStateFinish<SsrRadialReturn>>>
        reactions;

    static constexpr int total_iterations() {return 2;}
    static constexpr float ray_angle_increment_degree (){ return 90;}
    static constexpr float ray_length_meters(){ return 3;}

    int iteration_count = 0;

    static void onDefinition()
    {
        static_configure<KeyboardOrthogonal, SbKeyboard>();
    }

    void onInitialize()
    {
    }

    sc::result react(const smacc::EvStateFinish<SsrRadialReturn> &ev)
    {
        ROS_INFO("Superstate count: %d", iteration_count);
        ROS_INFO("RADIAL RETURN FINISH EVENT");
        if (++iteration_count == total_iterations()) // 1 == two times
        {
            
            ROS_INFO("Breaking radial motion");
            this->throwFinishEvent();
        }
        else
        {
            ROS_INFO("LOOPING TO Radial Rotate");
            return transit<SS1::SsrRadialRotate>();
        }
        

        return forward_event();
    }
};
//forward declaration for the superstate
using SS = SsRadialPattern1;

#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_rotate.h>
} // namespace SS1