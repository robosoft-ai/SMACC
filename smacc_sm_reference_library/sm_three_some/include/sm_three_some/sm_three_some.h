#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <sm_three_some/clients/cl_keyboard/cl_keyboard.h>

// ORTHOGONALS
#include <sm_three_some/orthogonals/or_timer.h>
#include <sm_three_some/orthogonals/or_updatable_publisher.h>
#include <sm_three_some/orthogonals/or_subscriber.h>
#include <sm_three_some/orthogonals/or_keyboard.h>

using namespace ros_timer_client;
using namespace ros_publisher_client;

using namespace sm_three_some::cl_subscriber;
using namespace sm_three_some::cl_keyboard;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>

#include <sm_three_some/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.h>
#include <sm_three_some/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.h>

#include <sm_three_some/clients/cl_keyboard/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>

//LOGIC UNITS
#include <all_events_go/lu_all_events_go.h>

using namespace smacc;

namespace sm_three_some
{
//SUPERSTATES
namespace SS1
{
class Ss1;
}

//STATES
class StState1; // first state specially needs a forward declaration
class StState2;
class StState3;

class MsRun;
class MsRecover;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmThreeSome
    : public smacc::SmaccStateMachineBase<SmThreeSome, MsRun>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrUpdatablePublisher>();
        this->createOrthogonal<OrKeyboard>();
    }
};
} // namespace sm_three_some

// MODE STATES
#include <sm_three_some/mode_states/ms_run.h>
#include <sm_three_some/mode_states/ms_recover.h>

//STATES
#include <sm_three_some/states/st_state_1.h>
#include <sm_three_some/states/st_state_2.h>
#include <sm_three_some/states/st_state_3.h>

#include <sm_three_some/superstates/ss_superstate_1.h>