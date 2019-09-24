#include <ros/ros.h>
#include <smacc/smacc.h>

//STATES
class StAcquireSensors;
class StRotateDegrees4;
class StNavigateForward1;
class StNavigateToWaypoint1;
class StNavigateToWaypointsX;
class StRotateDegrees2;
class StRotateDegrees1;
class StNavigateReverse2;
class StRotateDegrees3;
class StNavigateReverse1;

//SUPERSTATES
namespace SS1
{
    class SsRadialPattern1;
}

namespace SS2
{
    class SsRadialPattern2;
}

namespace SS3
{
    class SsRadialPattern3;
}

// STATE MACHINE
struct SmDanceBot
    : public smacc::SmaccStateMachineBase<SmDanceBot, StAcquireSensors>
{
    int counter_1;
    bool rt_ready_flag;

    SmDanceBot(my_context ctx, smacc::SignalDetector *signalDetector)
        : SmaccStateMachineBase<SmDanceBot, StAcquireSensors>(ctx, signalDetector)
    {
        this->setGlobalSMData("counter_1", counter_1);
        this->setGlobalSMData("rt_ready_flag", rt_ready_flag);
    }
};

// ORTHOGONALS
#include <sm_dance_bot/orthogonals/navigation_orthogonal.h>
#include <sm_dance_bot/orthogonals/obstacle_perception_orthogonal.h>
#include <sm_dance_bot/orthogonals/tool_orthogonal.h>
#include <sm_dance_bot/orthogonals/sensor_orthogonal.h>
#include <sm_dance_bot/orthogonals/keyboard_orthogonal.h>
#include <sm_dance_bot/orthogonals/publisher_orthogonal.h>


//SUBSTATE BEHAVIORS
#include <sm_dance_bot/substate_behaviors/timer/sb_timer_substate.h>

#include <sm_dance_bot/substate_behaviors/navigation/sb_rotate.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_undo_path_backwards.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_navigate_global_position.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_navigate_forward.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_navigate_backward.h>

#include <sm_dance_bot/substate_behaviors/temperature_sensor/sb_custom_condition_temperature_sensor.h>

#include <sm_dance_bot/substate_behaviors/tool/sb_tool_start.h>
#include <sm_dance_bot/substate_behaviors/tool/sb_tool_stop.h>

#include <sm_dance_bot/substate_behaviors/keyboard/sb_keyboard_substate.h>

#include <sm_dance_bot/substate_behaviors/publisher/sb_string_publisher.h>

#include <smacc/all_event_aggregator.h>
#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/LaserScan.h>

//class LidarSensor;
// COMPONENTS AND SUBSTATE BEHAVIORS FORWARD DECLARATIONS
using LidarSensor= smacc::SensorTopic<sensor_msgs::LaserScan>;

//STATES
#include <sm_dance_bot/states/st_acquire_sensors.h>
#include <sm_dance_bot/states/st_navigate_to_waypoints_x.h>

#include <sm_dance_bot/states/st_rotate_degrees_4.h>
#include <sm_dance_bot/states/st_navigate_forward_1.h>
#include <sm_dance_bot/states/st_navigate_to_waypoint_1.h>
#include <sm_dance_bot/states/st_rotate_degrees_2.h>
#include <sm_dance_bot/states/st_rotate_degrees_1.h>
#include <sm_dance_bot/states/st_navigate_reverse_2.h>
#include <sm_dance_bot/states/st_rotate_degrees_3.h>
#include <sm_dance_bot/states/st_navigate_reverse_1.h>

//SUPERSTATES
#include <sm_dance_bot/superstates/ss_radial_pattern_1.h>
#include <sm_dance_bot/superstates/ss_radial_pattern_2.h>
#include <sm_dance_bot/superstates/ss_radial_pattern_3.h>
