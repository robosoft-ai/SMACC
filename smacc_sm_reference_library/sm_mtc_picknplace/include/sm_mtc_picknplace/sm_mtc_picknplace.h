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
class StMoveToHome; // first state specially needs a forward declaration
class StMoveToPlace;
class StForbidCollisionObject;
class StLowerObject;
class StLiftObject;
class StAllowCollisionObject;
class StAttachObject;
class StCloseHand;
class StAllowCollisionHand;
class StGraspPoseIK;
class StGenerateGraspPose;
class StOpenHand;
class StEntryState;
class StMoveToPick;
class StApproachObject;
class StDetachObject;
class StRetreatAfterPlace;

class MsPickObject;
class MsPlaceObject;

// struct EvToDeep : sc::event<EvToDeep>{};

// struct EvFail : sc::event<EvFail>{};

// struct EvEStop : sc::event<EvEStop>{};

// STATE MACHINE
struct SmMTCPickNPlace
    : public smacc::SmaccStateMachineBase<SmMTCPickNPlace, StEntryState>
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
#include <sm_mtc_picknplace/mode_states/ms_pick_object.h>
#include <sm_mtc_picknplace/mode_states/ms_place_object.h>

//STATES
#include <sm_mtc_picknplace/states/st_move_to_home.h>
#include <sm_mtc_picknplace/states/st_move_to_place.h>
#include <sm_mtc_picknplace/states/st_open_hand.h>
#include <sm_mtc_picknplace/states/st_entry_state.h>
#include <sm_mtc_picknplace/states/st_move_to_pick.h>

#include <sm_mtc_picknplace/states/ms_pick_object_states/st_forbid_collision_object.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_lift_object.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_allow_collision_object.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_attach_object.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_close_hand.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_allow_collision_hand.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_grasp_pose_IK.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_generate_grasp_pose.h>
#include <sm_mtc_picknplace/states/ms_pick_object_states/st_approach_object.h>

#include <sm_mtc_picknplace/states/ms_place_object_states/st_detach_object.h>
#include <sm_mtc_picknplace/states/ms_place_object_states/st_forbid_collision_hand.h>
#include <sm_mtc_picknplace/states/ms_place_object_states/st_place_open_hand.h>
#include <sm_mtc_picknplace/states/ms_place_object_states/st_place_pose_IK.h>
#include <sm_mtc_picknplace/states/ms_place_object_states/st_generate_place_pose.h>
#include <sm_mtc_picknplace/states/ms_place_object_states/st_lower_object.h>
#include <sm_mtc_picknplace/states/ms_place_object_states/st_retreat_after_place.h>

