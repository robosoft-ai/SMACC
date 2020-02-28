#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_pr2_plugs/orthogonals/or_timer.h>
#include <sm_pr2_plugs/orthogonals/or_keyboard.h>

using namespace cl_ros_timer_client;
using namespace cl_keyboard;

//CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>


using namespace smacc;
using namespace cl_ros_timer_client;
using namespace smacc::default_events;

namespace sm_pr2_plugs{

//STATES
class StFailTuckArms;
class StFailUntuck;
class StFailLowerSpine;
class StSucceedFreeBase;
class StSucceedTuck;
class StStowPlug;
class StPullBackFromWall;
class StWiggleOut;
class StClearLeftArm;
class StCloseGripper;
class StRecoverStowPlug;
class StStowLeftArm;
class StFailOpenGripper;
class StPlugIn;
class StFetchPlug;
class StNavigate;
class StGetOutletLocations;
class StSafetyTuck;
class StUntuckAtOutlet;
class StGoalIsLocal;
class StFailStillUnplugged;
class StProcessRechargeCommand;
class StNavigateToOutlet;
class StUnplug;
class StDetectOutlet;

class MsRecharge;
class MsUnplug;

// struct EvToDeep : sc::event<EvToDeep>{};

// struct EvFail : sc::event<EvFail>{};

// struct EvEStop : sc::event<EvEStop>{};

// STATE MACHINE
struct SmPR2Plugs    : public smacc::SmaccStateMachineBase<SmPR2Plugs, MsRecharge>
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
#include <sm_pr2_plugs/mode_states/ms_recharge.h>
#include <sm_pr2_plugs/mode_states/ms_unplug.h>

//STATES
#include <sm_pr2_plugs/states/st_fail_tuck_arms.h>
#include <sm_pr2_plugs/states/st_fail_untuck.h>
#include <sm_pr2_plugs/states/st_fail_lower_spine.h>
#include <sm_pr2_plugs/states/st_succeed_free_base.h>
#include <sm_pr2_plugs/states/st_succeed_tuck.h>
#include <sm_pr2_plugs/states/st_stow_plug.h>
#include <sm_pr2_plugs/states/st_pull_back_from_wall.h>
#include <sm_pr2_plugs/states/st_wiggle_out.h>
#include <sm_pr2_plugs/states/st_clear_left_arm.h>
#include <sm_pr2_plugs/states/st_close_gripper.h>
#include <sm_pr2_plugs/states/st_recover_stow_plug.h>
#include <sm_pr2_plugs/states/st_stow_left_arm.h>
#include <sm_pr2_plugs/states/st_fail_open_gripper.h>
#include <sm_pr2_plugs/states/st_plug_in.h>
#include <sm_pr2_plugs/states/st_fetch_plug.h>
#include <sm_pr2_plugs/states/st_navigate.h>
#include <sm_pr2_plugs/states/st_get_outlet_locations.h>
#include <sm_pr2_plugs/states/st_safety_tuck.h>
#include <sm_pr2_plugs/states/st_untuck_at_outlet.h>
#include <sm_pr2_plugs/states/st_goal_is_local.h>
#include <sm_pr2_plugs/states/st_fail_still_unplugged.h>
#include <sm_pr2_plugs/states/st_process_recharge_command.h>
#include <sm_pr2_plugs/states/st_navigate_to_outlet.h>
#include <sm_pr2_plugs/states/st_unplug.h>
#include <sm_pr2_plugs/states/st_detect_outlet.h>
