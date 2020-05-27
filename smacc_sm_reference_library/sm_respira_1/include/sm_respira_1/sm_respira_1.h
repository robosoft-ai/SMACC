#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_respira_1/orthogonals/or_timer.h>
#include <sm_respira_1/orthogonals/or_updatable_publisher.h>
#include <sm_respira_1/orthogonals/or_subscriber.h>
#include <sm_respira_1/orthogonals/or_keyboard.h>

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
using namespace cl_keyboard;
using namespace sm_respira_1::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>

#include <sm_respira_1/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.h>
#include <sm_respira_1/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.h>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;

namespace sm_respira_1
{
//SUPERSTATES
namespace SS1
{
class SsACCycle;
} // namespace SS1

namespace SS2
{
class SsCMVCycle;
} // namespace SS2

namespace SS3
{
class SsPCCycle;
} // namespace SS3

namespace SS4
{
class SsPSCycle;
} // namespace SS4

//STATES
class StObserve;
class StLeakyLungStep1;
class StLeakyLungStep2;
class StLeakyLungStep3;
class StPatientObstructionStep1;
class StCalibrationStep1;
class StSystemShutdown;

//MODE STATES
class MsRun;
class MsLeakyLung;
class MsPatientObstruction;
class MsCalibration;
class MsShutdown;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmRespira1
    : public smacc::SmaccStateMachineBase<SmRespira1, MsRun>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrUpdatablePublisher>();
        this->createOrthogonal<OrKeyboard>();
        this->createOrthogonal<OrSubscriber>();
    }
};
} // namespace sm_respira_1

// MODE STATES
#include <sm_respira_1/mode_states/ms_run.h>
#include <sm_respira_1/mode_states/ms_leaky_lung.h>
#include <sm_respira_1/mode_states/ms_patient_obstruction.h>
#include <sm_respira_1/mode_states/ms_calibration.h>
#include <sm_respira_1/mode_states/ms_shutdown.h>


//STATES
#include <sm_respira_1/states/st_observe.h>
#include <sm_respira_1/states/st_leaky_lung_step_1.h>
#include <sm_respira_1/states/st_leaky_lung_step_2.h>
#include <sm_respira_1/states/st_leaky_lung_step_3.h>
#include <sm_respira_1/states/st_patient_obstruction_step_1.h>
#include <sm_respira_1/states/st_calibration_step_1.h>
#include <sm_respira_1/states/st_system_shutdown.h>

#include <sm_respira_1/superstates/ss_ac_cycle.h>
#include <sm_respira_1/superstates/ss_cmv_cycle.h>
#include <sm_respira_1/superstates/ss_pc_cycle.h>
#include <sm_respira_1/superstates/ss_ps_cycle.h>