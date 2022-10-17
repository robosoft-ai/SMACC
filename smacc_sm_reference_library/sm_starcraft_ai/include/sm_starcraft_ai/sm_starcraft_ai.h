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

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
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
#include <sr_all_events_go/sr_all_events_go.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;

namespace sm_starcraft_ai
{
//SUPERSTATES
namespace SS1
{
class SsMove;
} // namespace SS1

namespace SS2
{
class SsBuild;
} // namespace SS2

namespace SS3
{
class SsAttack;
} // namespace SS3

//STATES
class StObserve;

//MODE STATES
class MsRun;

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

//STATES
#include <sm_starcraft_ai/states/st_observe.h>

#include <sm_starcraft_ai/superstates/ss_move.h>
#include <sm_starcraft_ai/superstates/ss_build.h>
#include <sm_starcraft_ai/superstates/ss_attack.h>
