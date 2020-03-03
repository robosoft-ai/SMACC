#pragma once

#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_mtc_pour/orthogonals/or_timer.h>
#include <sm_mtc_pour/orthogonals/or_keyboard.h>

using namespace cl_ros_timer;
using namespace cl_keyboard;

//CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>


using namespace smacc;
using namespace cl_ros_timer;
using namespace smacc::default_events;

namespace sm_mtc_pour
{

//STATES
class StMoveHome; // first state specially needs a forward declaration
class StMoveToPrePlacePose;
class StPoseAboveGlass;
class StPutDownObject;
class StLiftObject;
class StPouring;
class StPrePourPose;
class StAllowGripperObjectCollision;
class StAllowGripperObjectCollision2;
class StAttachObject;
class StCloseGripper;
class StMoveToPrePourPose;
class StGraspPose;
class StGraspWorkspacePose;
class StOpenGripper;
class StEntryState;
class StMoveToPreGraspPose;
class StApproachObject;
class StDetachObject;
class StRetreatAfterPlace;

class MsGraspBottle;
class MsPlaceBottle;

// struct EvToDeep : sc::event<EvToDeep>{};

// struct EvFail : sc::event<EvFail>{};

// struct EvEStop : sc::event<EvEStop>{};

// STATE MACHINE
struct SmMTCPour
    : public smacc::SmaccStateMachineBase<SmMTCPour, StEntryState>
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
} // namespace sm_mtc_pour

// MODE STATES
#include <sm_mtc_pour/mode_states/ms_grasp_bottle.h>
#include <sm_mtc_pour/mode_states/ms_place_bottle.h>

//STATES
#include <sm_mtc_pour/states/st_move_home.h>
#include <sm_mtc_pour/states/st_move_to_pre_place_pose.h>
#include <sm_mtc_pour/states/st_open_gripper.h>
#include <sm_mtc_pour/states/st_entry_state.h>
#include <sm_mtc_pour/states/st_move_to_pre_grasp_pose.h>
#include <sm_mtc_pour/states/st_approach_object.h>
#include <sm_mtc_pour/states/st_grasp_workspace_pose.h>
#include <sm_mtc_pour/states/st_grasp_pose.h>

#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_pose_above_glass.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_lift_object.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_pouring.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_pre_pour_pose.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_allow_gripper_object_collision.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_attach_object.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_close_gripper.h>
#include <sm_mtc_pour/states/ms_grasp_bottle_states/st_move_to_pre_pour_pose.h>




#include <sm_mtc_pour/states/ms_place_bottle_states/st_detach_object.h>
#include <sm_mtc_pour/states/ms_place_bottle_states/st_allow_gripper_object_collision2.h>
#include <sm_mtc_pour/states/ms_place_bottle_states/st_release_object.h>
#include <sm_mtc_pour/states/ms_place_bottle_states/st_place_pose_kinematics.h>
#include <sm_mtc_pour/states/ms_place_bottle_states/st_place_pose.h>
#include <sm_mtc_pour/states/ms_place_bottle_states/st_put_down_object.h>
#include <sm_mtc_pour/states/ms_place_bottle_states/st_retreat_after_place.h>

