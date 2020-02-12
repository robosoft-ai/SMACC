#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_pr2_plugs/orthogonals/or_timer.h>
#include <sm_pr2_plugs/orthogonals/or_keyboard.h>

using namespace ros_timer_client;
using namespace keyboard_client;

//CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>


using namespace smacc;
using namespace ros_timer_client;
using namespace smacc::default_events;

namespace sm_pr2_plugs{

//STATES
class StFriday; // first state specially needs a forward declaration
class StMonday;
class StTuesday;
class StWednesday;
class StThursday;
class StSaturday;
class StSunday;

class MsWorkweek;
class MsWeekend;

// struct EvToDeep : sc::event<EvToDeep>{};

// struct EvFail : sc::event<EvFail>{};

// struct EvEStop : sc::event<EvEStop>{};

// STATE MACHINE
struct SmPR2Plugs    : public smacc::SmaccStateMachineBase<SmPR2Plugs, MsWorkweek>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
        // this->createOrthogonal<OrUpdatablePublisher>();
        this->createOrthogonal<OrKeyboard>();
        // this->createOrthogonal<OrSubscriber>();
    }
};
} // namespace sm_pr2_plugs
// MODE STATES
#include <sm_pr2_plugs/mode_states/ms_workweek.h>
#include <sm_pr2_plugs/mode_states/ms_weekend.h>

//STATES
#include <sm_pr2_plugs/states/st_friday.h>
#include <sm_pr2_plugs/states/st_monday.h>
#include <sm_pr2_plugs/states/st_tuesday.h>
#include <sm_pr2_plugs/states/st_wednesday.h>
#include <sm_pr2_plugs/states/st_thursday.h>
#include <sm_pr2_plugs/states/st_saturday.h>
#include <sm_pr2_plugs/states/st_sunday.h>
