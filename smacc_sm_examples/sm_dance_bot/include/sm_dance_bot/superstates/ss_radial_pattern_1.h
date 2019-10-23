
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
        //sc::transition<EvStateFinish<SsRadialPattern1>, StRotateDegrees1>,
        smacc::transition<EvStateFinish<SsRadialPattern1>, StRotateDegrees1>,
        //sc::custom_reaction<EvStateFinish<SsRadialPattern1>>,

        // Keyboard event
        smacc::transition<EvKeyPressN<SbKeyboard>, StRotateDegrees1>,
        smacc::transition<EvKeyPressP<SbKeyboard>, StNavigateToWaypointsX>,

        // Error events
        smacc::transition<smacc::EvTopicMessageTimeout<SbLidarSensor>, StAcquireSensors>,
        smacc::transition<EvActionAborted<smacc::SmaccMoveBaseActionClient>, StNavigateToWaypointsX>
        >
        
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

/*
    sc::result react(const EvStateFinish<SsRadialPattern1> &ev)
    {
        ROS_INFO("EV FINISHED FROM SUPERSTATE");
        return transit<StRotateDegrees1>();
    }
    */
};
//forward declaration for the superstate
using SS = SsRadialPattern1;

#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/radial_motion/ssr_radial_rotate.h>
} // namespace SS1