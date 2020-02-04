#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_packML/orthogonals/or_timer.h>
#include <sm_packML/orthogonals/or_updatable_publisher.h>
#include <sm_packML/orthogonals/or_subscriber.h>
#include <sm_packML/orthogonals/or_keyboard.h>

using namespace ros_timer_client;
using namespace ros_publisher_client;
using namespace keyboard_client;

using namespace sm_packML::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>

#include <sm_packML/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.h>
#include <sm_packML/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.h>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>

//STATE REACTORS
#include <all_events_go/sr_all_events_go.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;

namespace sm_packML
{
//SUPERSTATES
namespace SS1
{
class Ss1;
} // namespace SS1

//STATES
class StIdle; // first state specially needs a forward declaration
class StStarting;
class StExecute;
class StCompleting;
class StComplete;
class StResetting;
class StUnholding;
class StHeld;
class StHolding;
class StSuspending;
class StSuspended;
class StUnsuspending;
class StAborting;
class StAborted;
class StClearing;
class StStopping;
class StStopped;

class MsRun;
class MsRecover;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmPackML
    : public smacc::SmaccStateMachineBase<SmPackML, MsRun>
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
} // namespace sm_packML

// MODE STATES
#include <sm_packML/mode_states/ms_run.h>
#include <sm_packML/mode_states/ms_recover.h>

//STATES
#include <sm_packML/states/st_idle.h>
#include <sm_packML/states/st_starting.h>
#include <sm_packML/states/st_execute.h>
#include <sm_packML/states/st_completing.h>
#include <sm_packML/states/st_complete.h>
#include <sm_packML/states/st_resetting.h>
#include <sm_packML/states/st_unholding.h>
#include <sm_packML/states/st_held.h>
#include <sm_packML/states/st_holding.h>
#include <sm_packML/states/st_suspending.h>
#include <sm_packML/states/st_suspended.h>
#include <sm_packML/states/st_unsuspending.h>
#include <sm_packML/states/st_aborting.h>
#include <sm_packML/states/st_aborted.h>
#include <sm_packML/states/st_clearing.h>
#include <sm_packML/states/st_stopping.h>
#include <sm_packML/states/st_stopped.h>

#include <sm_packML/superstates/ss_superstate_1.h>