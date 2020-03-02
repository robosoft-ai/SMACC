#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_starcraft_ai/orthogonals/or_timer.h>
#include <sm_starcraft_ai/orthogonals/or_updatable_publisher.h>
#include <sm_starcraft_ai/orthogonals/or_subscriber.h>
#include <sm_starcraft_ai/orthogonals/or_keyboard.h>

using namespace cl_ros_timer_client;
using namespace cl_ros_publisher_client;
using namespace cl_keyboard;

using namespace sm_starcraft_ai::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>

#include <sm_starcraft_ai/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.h>
#include <sm_starcraft_ai/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.h>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

//STATE REACTORS
#include <all_events_go/sr_all_events_go.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;

namespace sm_starcraft_ai
{
//SUPERSTATES
namespace SS1
{
class Ss1;
} // namespace SS1

namespace SS2
{
class Ss2;
} // namespace SS2

namespace SS3
{
class Ss3;
} // namespace SS3


//STATES
class StState1; // first state specially needs a forward declaration
class StState2;
class StState3;
class StState4;

class MsRun;
class MsRecover;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmStarcraftAI
    : public smacc::SmaccStateMachineBase<SmStarcraftAI, MsRun>
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
} // namespace sm_starcraft_ai

// MODE STATES
#include <sm_starcraft_ai/mode_states/ms_run.h>
#include <sm_starcraft_ai/mode_states/ms_recover.h>

//STATES
#include <sm_starcraft_ai/states/st_state_1.h>
#include <sm_starcraft_ai/states/st_state_2.h>
#include <sm_starcraft_ai/states/st_state_3.h>
#include <sm_starcraft_ai/states/st_state_4.h>

#include <sm_starcraft_ai/superstates/ss_superstate_1.h>
#include <sm_starcraft_ai/superstates/ss_superstate_2.h>
#include <sm_starcraft_ai/superstates/ss_superstate_3.h>