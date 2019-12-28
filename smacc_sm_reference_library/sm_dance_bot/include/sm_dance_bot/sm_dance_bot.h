#include <smacc/smacc.h>

#include <sensor_msgs/LaserScan.h>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_ros_timer.h>

#include <smacc_interface_components/client_behaviors/cb_sensor_base.h>

#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
using namespace move_base_z_client;

#include <sm_dance_bot/clients/move_base_z_client/client_behaviors/cb_rotate.h>
#include <sm_dance_bot/clients/move_base_z_client/client_behaviors/cb_undo_path_backwards.h>
#include <sm_dance_bot/clients/move_base_z_client/client_behaviors/cb_navigate_global_position.h>
#include <sm_dance_bot/clients/move_base_z_client/client_behaviors/cb_navigate_forward.h>
#include <sm_dance_bot/clients/move_base_z_client/client_behaviors/cb_navigate_backward.h>

#include <sm_dance_bot/clients/tool_client/client_behaviors/cb_tool_start.h>
#include <sm_dance_bot/clients/tool_client/client_behaviors/cb_tool_stop.h>

#include <sm_dance_bot/clients/lidar_client/client_behaviors/cb_lidar_sensor.h>
#include <sm_dance_bot/clients/temperature_sensor_client/client_behaviors/cb_custom_condition_temperature_sensor.h>

#include <sm_dance_bot/clients/string_publisher_client/client_behaviors/cb_string_publisher.h>
#include <sm_dance_bot/clients/service3_client/client_behaviors/cb_service3.h>
#include <sm_dance_bot/clients/updatable_publisher_client/client_behaviors/cb_updatable_publisher.h>



//LOGIC UNITS
#include <all_events_go/lu_all_events_go.h>
#include <event_countdown/event_countdown.h>
#include <conditional/lu_conditional.h>

using namespace sm_dance_bot::lidar_client;
using namespace sm_dance_bot::move_base_z_client;
using namespace sm_dance_bot::service3_client;
using namespace sm_dance_bot::string_publisher_client;
using namespace sm_dance_bot::temperature_sensor_client;
using namespace sm_dance_bot::tool_client;
using namespace sm_dance_bot::updatable_publisher_client;

// ORTHOGONALS
#include <sm_dance_bot/orthogonals/or_navigation.h>
#include <sm_dance_bot/orthogonals/or_obstacle_perception.h>
#include <sm_dance_bot/orthogonals/or_tool.h>
#include <sm_dance_bot/orthogonals/or_temperature_sensor.h>
#include <sm_dance_bot/orthogonals/or_string_publisher.h>
#include <sm_dance_bot/orthogonals/or_service3.h>
#include <sm_dance_bot/orthogonals/or_timer.h>
#include <sm_dance_bot/orthogonals/or_updatable_publisher.h>

namespace sm_dance_bot
{
//STATE FORWARD DECLARATIONS
class StAcquireSensors;
class StEventCountDown;
class StRotateDegrees4;
class StNavigateForward1;
class StNavigateToWaypoint1;
class StNavigateToWaypointsX;
class StRotateDegrees2;
class StRotateDegrees1;
class StNavigateReverse2;
class StRotateDegrees3;
class StNavigateReverse1;
class StNavigateForward2;
class StRotateDegrees5;
class StNavigateReverse3;
class StRotateDegrees6;
class StNavigateReverse3;

//SUPERSTATE FORWARD DECLARATIONS

// MEGASTATE FORWARD DECLARATIONS
class MsDanceBotRunMode;
class MsDanceBotRecoveryMode;

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

namespace SS4
{
class SsFPattern1;
}

namespace SS5
{
class SsSPattern1;
}

// custom smd_dance_bot event
struct EvGlobalError : sc::event<EvGlobalError>
{
};

} // namespace sm_dance_bot

using namespace sm_dance_bot;
using namespace ros_timer_client;
using namespace smacc;

namespace sm_dance_bot
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmDanceBot
    : public smacc::SmaccStateMachineBase<SmDanceBot, MsDanceBotRunMode>
{
    int counter_1;
    bool rt_ready_flag;

    typedef mpl::bool_<false> shallow_history;
    typedef mpl::bool_<false> deep_history;
    typedef mpl::bool_<false> inherited_deep_history;

    typedef smacc::SmaccStateMachineBase<SmDanceBot, MsDanceBotRunMode> base;

    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->setGlobalSMData("counter_1", counter_1);
        this->setGlobalSMData("rt_ready_flag", rt_ready_flag);

        this->createOrthogonal<OrNavigation>();
        this->createOrthogonal<OrObstaclePerception>();
        this->createOrthogonal<OrTool>();
        this->createOrthogonal<OrTemperatureSensor>();
        this->createOrthogonal<OrStringPublisher>();
        this->createOrthogonal<OrService3>();
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrUpdatablePublisher>();
    }
};

} // namespace sm_dance_bot

//MEGASTATES
#include <sm_dance_bot/modestates/ms_dance_bot_run_mode.h>
#include <sm_dance_bot/modestates/ms_dance_bot_recovery_mode.h>

//SUPERSTATES
#include <sm_dance_bot/superstates/ss_radial_pattern_1.h>
#include <sm_dance_bot/superstates/ss_radial_pattern_2.h>
#include <sm_dance_bot/superstates/ss_radial_pattern_3.h>
#include <sm_dance_bot/superstates/ss_f_pattern_1.h>
#include <sm_dance_bot/superstates/ss_s_pattern_1.h>

//STATES
#include <sm_dance_bot/states/st_acquire_sensors.h>
#include <sm_dance_bot/states/st_event_count_down.h>

#include <sm_dance_bot/states/st_navigate_to_waypoints_x.h>

#include <sm_dance_bot/states/st_rotate_degrees_6.h>
#include <sm_dance_bot/states/st_rotate_degrees_5.h>
#include <sm_dance_bot/states/st_navigate_forward_2.h>
#include <sm_dance_bot/states/st_rotate_degrees_4.h>
#include <sm_dance_bot/states/st_navigate_forward_1.h>
#include <sm_dance_bot/states/st_navigate_to_waypoint_1.h>
#include <sm_dance_bot/states/st_rotate_degrees_2.h>
#include <sm_dance_bot/states/st_rotate_degrees_1.h>
#include <sm_dance_bot/states/st_navigate_reverse_2.h>
#include <sm_dance_bot/states/st_rotate_degrees_3.h>
#include <sm_dance_bot/states/st_navigate_reverse_1.h>
#include <sm_dance_bot/states/st_navigate_reverse_3.h>