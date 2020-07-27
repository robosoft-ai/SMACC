#include <smacc/smacc.h>

namespace sm_ridgeback_floor_coverage_dynamic_1
{
namespace SS5
{
namespace sm_ridgeback_floor_coverage_dynamic_1
{
namespace s_pattern_states
{

// FORWARD DECLARATION OF INNER STATES
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
} // namespace sm_ridgeback_floor_coverage_dynamic_1

enum class TDirection
{
    LEFT,
    RIGHT
};

using namespace sm_ridgeback_floor_coverage_dynamic_1::s_pattern_states;

// STATE DECLARATION
struct SsSPattern1 : smacc::SmaccState<SsSPattern1, MsDanceBotRunMode, StiSPatternLoopStart>
{
public:
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvLoopEnd<StiSPatternLoopStart>, StNavigateToWaypointsX, ENDLOOP>

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    static constexpr float pitch1_lenght_meters() { return 0.6; }
    static constexpr float pitch2_lenght_meters() { return 1.75; }
    static constexpr int total_iterations() { return 12; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }
    double initialStateAngle;

    int iteration_count;

    void runtimeConfigure()
    {
        
        this->iteration_count = 0;

        cl_move_base_z::ClMoveBaseZ *robot;
        this->requiresClient(robot);

        if (robot != nullptr)
        {
            auto pose = robot->getComponent<cl_move_base_z::Pose>()->toPoseMsg();
            this->initialStateAngle = angles::to_degrees(angles::normalize_angle(tf::getYaw(pose.orientation)));
            ROS_INFO("Initial angle for F pattern: %lf", initialStateAngle);
        }
        else
        {
            ROS_ERROR("robot pose not found to plan the FPattern motion");
        }
    }
};

// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsSPattern1;
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_rotate_1.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_forward_1.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_rotate_2.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_forward_2.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_rotate_3.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_forward_3.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_rotate_4.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_forward_4.h>
#include <sm_ridgeback_floor_coverage_dynamic_1/states/s_pattern_states/sti_spattern_loop_start.h>

} // namespace SS5
} // namespace sm_ridgeback_floor_coverage_dynamic_1