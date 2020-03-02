#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_mtc_picknplace/orthogonals/or_timer.h>
#include <sm_mtc_picknplace/orthogonals/or_keyboard.h>

using namespace cl_ros_timer_client;
using namespace cl_keyboard;

//CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>


using namespace smacc;
using namespace cl_ros_timer_client;
using namespace smacc::default_events;

namespace sm_mtc_picknplace
{

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
struct SmMTCPickNPlace
    : public smacc::SmaccStateMachineBase<SmMTCPickNPlace, MsWorkweek>
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
} // namespace sm_mtc_picknplace

// MODE STATES
#include <sm_mtc_picknplace/mode_states/ms_workweek.h>
#include <sm_mtc_picknplace/mode_states/ms_weekend.h>

//STATES
#include <sm_mtc_picknplace/states/st_friday.h>
#include <sm_mtc_picknplace/states/st_monday.h>
#include <sm_mtc_picknplace/states/st_tuesday.h>
#include <sm_mtc_picknplace/states/st_wednesday.h>
#include <sm_mtc_picknplace/states/st_thursday.h>
#include <sm_mtc_picknplace/states/st_saturday.h>
#include <sm_mtc_picknplace/states/st_sunday.h>
