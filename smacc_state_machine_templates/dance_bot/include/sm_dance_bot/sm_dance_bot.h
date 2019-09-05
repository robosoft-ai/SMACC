#include <ros/ros.h>
#include <smacc_core/smacc.h>


//STATES
class st_acquire_sensors;
class st_rotate_degrees_4;
class st_navigate_forward_1;
class st_navigate_to_waypoint_1;
class st_navigate_to_waypoints_x;
class st_rotate_degrees_2;
class st_rotate_degrees_1;
class st_navigate_reverse_2;
class st_rotate_degrees_3;
class st_navigate_reverse_1;

//SUPERSTATES
namespace SS1
{
    class ss_radial_pattern_1;
}

namespace SS2
{
    class ss_radial_pattern_2;
}

namespace SS3
{
    class ss_radial_pattern_3;
}

// STATE MACHINE
struct sm_dance_bot
    : public smacc::SmaccStateMachineBase<sm_dance_bot, st_acquire_sensors>
{
    sm_dance_bot(my_context ctx, smacc::SignalDetector *signalDetector)
        : SmaccStateMachineBase<sm_dance_bot, st_acquire_sensors>(ctx, signalDetector)
    {
    }
};

// ORTHOGONALS
#include <sm_dance_bot/orthogonals/navigation_orthogonal.h>
#include <sm_dance_bot/orthogonals/obstacle_perception_orthogonal.h>
#include <sm_dance_bot/orthogonals/tool_orthogonal.h>
#include <sm_dance_bot/orthogonals/sensor_orthogonal.h>


//SUBSTATE BEHAVIORS
#include <sm_dance_bot/substate_behaviors/timer/sb_timer_substate.h>

#include <sm_dance_bot/substate_behaviors/navigation/sb_rotate.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_undo_path_backwards.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_navigate_global_position.h>
#include <sm_dance_bot/substate_behaviors/navigation/sb_navigate_forward.h>

#include <sm_dance_bot/substate_behaviors/temperature_sensor/sb_custom_condition_temperature_sensor.h>

#include <sm_dance_bot/substate_behaviors/tool/sb_tool_start.h>
#include <sm_dance_bot/substate_behaviors/tool/sb_tool_stop.h>

#include <sm_dance_bot/substate_behaviors/keyboard/sb_keyboard_substate.h>

#include <smacc_core/all_event_aggregator.h>
#include <smacc_interface_components/substate_behaviors/sensor_substate.h>
#include <sensor_msgs/LaserScan.h>

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
