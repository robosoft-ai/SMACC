#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_calendar_week/orthogonals/or_timer.h>
#include <sm_calendar_week/orthogonals/or_keyboard.h>

using namespace cl_ros_timer;
using namespace cl_keyboard;

//CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>


using namespace smacc;
using namespace cl_ros_timer;
using namespace smacc::default_events;

namespace sm_calendar_week
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
struct SmCalendarWeek
    : public smacc::SmaccStateMachineBase<SmCalendarWeek, MsWorkweek>
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
} // namespace sm_calendar_week

// MODE STATES
#include <sm_calendar_week/mode_states/ms_workweek.h>
#include <sm_calendar_week/mode_states/ms_weekend.h>

//STATES
#include <sm_calendar_week/states/st_friday.h>
#include <sm_calendar_week/states/st_monday.h>
#include <sm_calendar_week/states/st_tuesday.h>
#include <sm_calendar_week/states/st_wednesday.h>
#include <sm_calendar_week/states/st_thursday.h>
#include <sm_calendar_week/states/st_saturday.h>
#include <sm_calendar_week/states/st_sunday.h>
