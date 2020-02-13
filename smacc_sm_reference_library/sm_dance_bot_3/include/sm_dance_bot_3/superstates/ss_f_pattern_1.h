#include <smacc/smacc.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

namespace sm_dance_bot_3
{
namespace f_pattern_states
{

enum class TDirection
{
    LEFT,
    RIGHT
};

// FORWARD DECLARATIONS OF INNER STATES
template <typename SS>
class StiFPatternRotate1;
template <typename SS>
class StiFPatternForward1;
template <typename SS>
class StiFPatternReturn1;
template <typename SS>
class StiFPatternRotate2;
template <typename SS>
class StiFPatternForward2;
template <typename SS>
class StiFPatternStartLoop;
} // namespace f_pattern_states
} // namespace sm_dance_bot_3
namespace sm_dance_bot_3
{
namespace SS4
{
using namespace f_pattern_states;

// STATE DECLARATION
struct SsFPattern1 : smacc::SmaccState<SsFPattern1, MsDanceBotRunMode, StiFPatternStartLoop<SsFPattern1>>
{
public:
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvLoopEnd<StiFPatternStartLoop<SsFPattern1>>, StNavigateToWaypointsX, ENDLOOP>>
        reactions;

    // STATE VARIABLES
    // superstate parameters
    static constexpr float ray_lenght_meters() { return 3.25; }
    static constexpr float pitch_lenght_meters() { return 0.6; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    double initialStateAngle = 0;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
        move_base_z_client::ClMoveBaseZ *robot;
        this->requiresClient(robot);

        if (robot != nullptr)
        {
            auto pose = robot->getComponent<move_base_z_client::Pose>()->get();
            this->initialStateAngle = angles::to_degrees(angles::normalize_angle (tf::getYaw(pose.orientation) ));
            ROS_INFO("Initial angle for F pattern: %lf", initialStateAngle);
        }
        else
        {
            ROS_ERROR("robot pose not found to plan the FPattern motion");
        }
    }
}; // namespace SS4

// FORWARD DECLARATION FOR THE SUPERSTATE
} // namespace SS4
} // namespace sm_dance_bot_3
#include <sm_dance_bot_3/states/f_pattern_states/sti_fpattern_rotate_1.h>
#include <sm_dance_bot_3/states/f_pattern_states/sti_fpattern_forward_1.h>
#include <sm_dance_bot_3/states/f_pattern_states/sti_fpattern_return_1.h>
#include <sm_dance_bot_3/states/f_pattern_states/sti_fpattern_rotate_2.h>
#include <sm_dance_bot_3/states/f_pattern_states/sti_fpattern_forward_2.h>
#include <sm_dance_bot_3/states/f_pattern_states/sti_fpattern_loop_start.h>
