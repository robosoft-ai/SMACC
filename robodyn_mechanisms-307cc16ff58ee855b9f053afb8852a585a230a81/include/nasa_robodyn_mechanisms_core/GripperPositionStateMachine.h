#ifndef GRIPPERPOSITIONSTATEMACHINE_H
#define GRIPPERPOSITIONSTATEMACHINE_H

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/in_state_reaction.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include "nasa_common_utilities/Logger.h"
#include <nasa_r2_common_msgs/JointControlData.h>
#include <sensor_msgs/JointState.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nasa_r2_common_msgs/JointCommand.h>
#include <boost/regex.hpp>
#include "nasa_robodyn_mechanisms_core/GripperKinematics.h"
#include "nasa_robodyn_utilities/AbsSlewFilter.h"
#include "nasa_r2_config_core/GripperSetpointFactory.h"
#include "nasa_common_utilities/StatechartUtilities.h"

//! EELockedInitValue

//! Events
//! Nomenclature:
//!    c_whatever - _c_ommand from a user
//!    e_whatever - _e_vent from a state
//!    i_whatever - _i_nternal use only, for acting the same event across orthogonal states
struct e_do                                : boost::statechart::event<e_do> {};
    struct i_do_movement                   : boost::statechart::event<i_do_movement> {};
    struct i_do_active                     : boost::statechart::event<i_do_active> {};
    struct i_do_load                       : boost::statechart::event<i_do_load> {};
    struct i_do_lock                       : boost::statechart::event<i_do_lock> {};
    struct i_do_lockStatus                 : boost::statechart::event<i_do_lockStatus> {};
    struct i_do_status                     : boost::statechart::event<i_do_status> {};
    struct i_do_position                   : boost::statechart::event<i_do_position> {};
    struct i_do_positionOrLess             : boost::statechart::event<i_do_positionOrLess> {};
    struct i_do_stall                      : boost::statechart::event<i_do_stall> {};
    struct i_do_fault                      : boost::statechart::event<i_do_fault> {};
    struct i_do_moving                     : boost::statechart::event<i_do_moving> {};
    struct i_do_open                       : boost::statechart::event<i_do_open> {};
    struct i_do_force                      : boost::statechart::event<i_do_force> {};
struct e_loaded                            : boost::statechart::event<e_loaded> {};
struct e_unloaded                          : boost::statechart::event<e_unloaded> {};
struct e_locked                            : boost::statechart::event<e_locked> {};
    struct i_locked_action                 : boost::statechart::event<i_locked_action> {};
    struct i_locked_lock                   : boost::statechart::event<i_locked_lock> {};
struct e_unlocked                          : boost::statechart::event<e_unlocked> {};
struct e_lockedStatus                      : boost::statechart::event<e_lockedStatus> {};
struct e_unlockedStatus                    : boost::statechart::event<e_unlockedStatus> {};
struct e_ready                             : boost::statechart::event<e_ready> {};
    struct i_ready_command                 : boost::statechart::event<i_ready_command> {};
    struct i_ready_status                  : boost::statechart::event<i_ready_status> {};
struct e_unready                           : boost::statechart::event<e_unready> {};
    struct i_unready_command               : boost::statechart::event<i_unready_command> {};
    struct i_unready_status                : boost::statechart::event<i_unready_status> {};
struct e_lessThanDesiredJawPosition        : boost::statechart::event<e_lessThanDesiredJawPosition> {};
struct e_atDesiredJawPosition              : boost::statechart::event<e_atDesiredJawPosition> {};
    struct i_atDesiredJawPosition_action   : boost::statechart::event<i_atDesiredJawPosition_action> {};
    struct i_atDesiredJawPosition_position : boost::statechart::event<i_atDesiredJawPosition_position> {};
struct e_moreThanDesiredJawPosition        : boost::statechart::event<e_moreThanDesiredJawPosition> {};
struct e_faulted                           : boost::statechart::event<e_faulted> {};
    struct i_faulted_command               : boost::statechart::event<i_faulted_command> {};
    struct i_faulted_fault                 : boost::statechart::event<i_faulted_fault> {};
struct e_notFaulted                        : boost::statechart::event<e_notFaulted> {};
struct e_stalled                           : boost::statechart::event<e_stalled> {};
    struct i_stalled_action                : boost::statechart::event<i_stalled_action> {};
    struct i_stalled_stall                 : boost::statechart::event<i_stalled_stall> {};
struct e_notStalled                        : boost::statechart::event<e_notStalled> {};
struct e_stop                              : boost::statechart::event<e_stop> {};
struct e_open                              : boost::statechart::event<e_open> {};
struct e_close                             : boost::statechart::event<e_close> {};
struct e_isOpen                            : boost::statechart::event<e_isOpen> {};
    struct i_isOpen_movement               : boost::statechart::event<i_isOpen_movement> {};
    struct i_isOpen_action                 : boost::statechart::event<i_isOpen_action> {};
    struct i_isOpen_open                   : boost::statechart::event<i_isOpen_open> {};
struct e_isNotOpen                         : boost::statechart::event<e_isNotOpen> {};
struct e_isClosed                          : boost::statechart::event<e_isClosed> {};
struct e_still                             : boost::statechart::event<e_still> {};
    struct i_still_action                  : boost::statechart::event<i_still_action> {};
    struct i_still_moving                  : boost::statechart::event<i_still_moving> {};
struct e_moving                            : boost::statechart::event<e_moving> {};
struct e_lessThanForceThreshold            : boost::statechart::event<e_lessThanForceThreshold> {};
struct e_moreThanForceThreshold            : boost::statechart::event<e_moreThanForceThreshold> {};
struct e_done                              : boost::statechart::event<e_done> {};
struct c_cancel                            : boost::statechart::event<c_cancel> {};
struct c_set                               : boost::statechart::event<c_set> {};
struct e_set                               : boost::statechart::event<e_set> {};
struct c_lock                              : boost::statechart::event<c_lock> {};
struct e_lock                              : boost::statechart::event<e_lock> {};
struct c_release                           : boost::statechart::event<c_release> {};
struct e_release                           : boost::statechart::event<e_release> {};
struct e_listen                            : boost::statechart::event<e_listen> {};
struct e_next                              : boost::statechart::event<e_next> {};
struct c_setParameters                     : boost::statechart::event<c_setParameters>
{
    GripperSetpoint sp;
    c_setParameters(const GripperSetpoint& setPoint) : sp(setPoint) {}
};

//! Forward Declaration of States
struct Inactive;
struct Active;
struct Stopped;
struct Opening;
struct Closing;
struct Listening;
struct Acting;
struct OpenWide;
struct JawPositionCheck;
struct Close;
struct LockCheck;
struct OpenUntilStall;
struct Done;
struct Unloaded;
struct Loaded;
struct Unlocked;
struct Locked;
struct UnlockedStatus;
struct LockedStatus;
struct Unready;
struct Ready;
struct LessThanDesiredJawPosition;
struct AtDesiredJawPosition;
struct MoreThanDesiredJawPosition;
struct NotStalled;
struct Stalled;
struct NotFaulted;
struct Faulted;
struct Still;
struct Moving;
struct NotOpen;
struct Open;
struct LessThanForceThreshold;
struct MoreThanForceThreshold;
struct Container;

//! State Machine Definition
struct GripperPosition : boost::statechart::state_machine<GripperPosition, Container>
{
    GripperPosition();

    std::string mechanism;

    nasa_r2_common_msgs::JointControlData jointControlData;
    sensor_msgs::JointState encoderState, jawLeftState, jawRightState;
    nasa_r2_common_msgs::JointCommand jointCommand;
    double currentMax;
    uint16_t dutyCycleMax;

    boost::function<float(const std::string&)>          getFloat;
    boost::function<void(const std::string&, float)>    setFloat;
    boost::function<uint16_t(const std::string&)>       getUInt16;
    boost::function<void(const std::string&, uint16_t)> setUInt16;
    boost::function<void(const nasa_r2_common_msgs::JointCommand& msg)> sendCommand;
    boost::function<void(const actionlib_msgs::GoalStatus& status, bool clear)> sendGoalStatus;
    actionlib_msgs::GoalStatus goalStatus;

    std::string dutyCycleLimitName;
    uint16_t dutyCycleLimit;
    std::string currentLimitName;
    double currentLimit;
    std::string measuredCurrentName;
    double measuredCurrent;

    double jawOpenPosition;
    double jawPositionNoise;
    double encoderOpenVelocity;
    double encoderCloseVelocity;
    double encoderClosedPosition;
    double encoderPositionNoise;
    double encoderVelocityNoise;
    double currentNoise;
    double weakForce;
    double strongForce;
    double desiredForce;
    double jawPositionForceThreshold;
    AbsSlewFilter outputCurrentFilter;
    AbsSlewFilter dutyCycleLimitFilter;

    std::string initializationStateName;

    GripperSetpoint setPoint;
    GripperKinematicsPtr gripperKinematics;
};

struct Container : boost::statechart::simple_state<Container, GripperPosition,
                                                   boost::mpl::list<Inactive, Locked, LockedStatus, Unloaded, LessThanDesiredJawPosition, NotStalled, Unready, NotFaulted, Still, NotOpen, LessThanForceThreshold> >
{
    void doo(const e_do&);
    void ready(const e_ready&);
    void unready(const e_unready&);
    void locked(const e_locked&);
    void atDesiredJawPosition(const e_atDesiredJawPosition&);
    void stalled(const e_stalled&);
    void faulted(const e_faulted&);
    void still(const e_still&);
    void isOpen(const e_isOpen&);
    void setParameters(const c_setParameters& e);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< e_do, Container, &Container::doo >,
                              boost::statechart::in_state_reaction< e_ready, Container, &Container::ready >,
                              boost::statechart::in_state_reaction< e_unready, Container, &Container::unready >,
                              boost::statechart::in_state_reaction< e_locked, Container, &Container::locked >,
                              boost::statechart::in_state_reaction< e_atDesiredJawPosition, Container, &Container::atDesiredJawPosition >,
                              boost::statechart::in_state_reaction< e_stalled, Container, &Container::stalled >,
                              boost::statechart::in_state_reaction< e_faulted, Container, &Container::faulted >,
                              boost::statechart::in_state_reaction< e_still, Container, &Container::still >,
                              boost::statechart::in_state_reaction< e_isOpen, Container, &Container::isOpen > > reactions;
};

//! has to be a boost::statechart::state because of the context<> in the constructor
struct Inactive : boost::statechart::state<Inactive, Container::orthogonal<0> >
{
    Inactive(my_context ctx);
    boost::statechart::result react(const i_ready_command&);

    typedef boost::mpl::list< boost::statechart::custom_reaction< i_ready_command>,
                              boost::statechart::transition< c_setParameters, Inactive, Container, &Container::setParameters> > reactions;

};

//! has to be a boost::statechart::state because of the context<> in the constructor
struct Active : boost::statechart::state<Active, Container::orthogonal<0>, boost::mpl::list<Stopped, Listening> >
{
    Active(my_context ctx);

    void doo(const i_do_active&);
    void set(const c_set&);
    void lock(const c_lock&);
    void release(const c_release&);
    void cancel(const c_cancel&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_active, Active, &Active::doo >,
                              boost::statechart::in_state_reaction< c_set, Active, &Active::set >,
                              boost::statechart::in_state_reaction< c_lock, Active, &Active::lock >,
                              boost::statechart::in_state_reaction< c_release, Active, &Active::release >,
                              boost::statechart::in_state_reaction< c_cancel, Active, &Active::cancel >,
                              boost::statechart::transition< i_unready_command, Inactive >, 
                              boost::statechart::transition< i_faulted_command, Inactive > > reactions;

    double desiredCurrent;
    double outputCurrent;
};

//! has to be a boost::statechart::state because of the context<> in the constructor
struct Stopped : boost::statechart::state<Stopped, Active::orthogonal<0> >
{
    Stopped(my_context ctx);

    typedef boost::mpl::list< boost::statechart::transition< e_open,  Opening >, 
                              boost::statechart::transition< e_close, Closing > > reactions;

};

struct Opening : boost::statechart::simple_state<Opening, Active::orthogonal<0> >
{
    void doo(const i_do_movement&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_movement, Opening, &Opening::doo >,
                              boost::statechart::transition< e_stop, Stopped >,
                              boost::statechart::transition< i_isOpen_movement, Stopped >, 
                              boost::statechart::transition< e_close, Closing > > reactions;
};

struct Closing : boost::statechart::simple_state<Closing, Active::orthogonal<0> >
{
    void doo(const i_do_movement&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_movement, Closing, &Closing::doo >,
                              boost::statechart::transition< e_stop,     Stopped >,
                              boost::statechart::transition< e_isClosed, Stopped >, 
                              boost::statechart::transition< e_open,     Opening > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct Listening : boost::statechart::state<Listening, Active::orthogonal<1> >
{
    Listening(my_context ctx);

    typedef boost::mpl::list< boost::statechart::transition< e_set,           OpenWide >,
                              boost::statechart::transition< e_lock,          Close >, 
                              boost::statechart::transition< e_release,       OpenUntilStall >,
                              boost::statechart::transition< c_setParameters, Listening, Container, &Container::setParameters> > reactions;
};

//! Done is the initial internal transition only as a default because there has to be something, and it seems like the one to cause the least problems.
struct Acting : boost::statechart::simple_state<Acting, Active::orthogonal<1>, Done >
{
    typedef boost::mpl::list< boost::statechart::transition< e_listen, Listening > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct OpenWide : boost::statechart::state<OpenWide, Acting >
{
    OpenWide(my_context ctx);
    boost::statechart::result react(const i_stalled_action&);

    typedef boost::mpl::list< boost::statechart::custom_reaction< i_stalled_action >,
                              boost::statechart::transition< i_isOpen_action, JawPositionCheck>, 
                              boost::statechart::transition< e_next, JawPositionCheck > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct JawPositionCheck : boost::statechart::state<JawPositionCheck, Acting >
{
    JawPositionCheck(my_context ctx);
    boost::statechart::result react(const i_stalled_action&);

    typedef boost::mpl::list< boost::statechart::transition< i_atDesiredJawPosition_action, Done >,
                              boost::statechart::transition< e_next, Done >,
                              boost::statechart::custom_reaction< i_stalled_action > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct Close : boost::statechart::state<Close, Acting >
{
    Close(my_context ctx);
    boost::statechart::result react(const i_stalled_action&);

    typedef boost::mpl::list< boost::statechart::transition< i_atDesiredJawPosition_action, LockCheck >,
                              boost::statechart::transition< e_next, LockCheck >,
                              boost::statechart::custom_reaction< i_stalled_action > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct LockCheck : boost::statechart::state<LockCheck, Acting >
{
    LockCheck(my_context ctx);
    boost::statechart::result react(const i_stalled_action&);

    typedef boost::mpl::list< boost::statechart::transition< e_next, Done >,
                              boost::statechart::custom_reaction< i_stalled_action > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct OpenUntilStall : boost::statechart::state<OpenUntilStall, Acting >
{
    OpenUntilStall(my_context ctx);

    typedef boost::mpl::list< boost::statechart::transition< i_stalled_action, Done >,
                              boost::statechart::transition< e_next, Done >,
                              boost::statechart::transition< i_isOpen_action, Done > > reactions;
};

//! has to be a boost::statechart::state because of the post_event in the constructor
struct Done : boost::statechart::state<Done, Acting >
{
    Done(my_context ctx);

    typedef boost::statechart::transition< e_done, Listening > reactions;
};

struct Unlocked : boost::statechart::simple_state<Unlocked, Container::orthogonal<1> >
{
    void doo(const i_do_lock&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_lock, Unlocked, &Unlocked::doo >,
                              boost::statechart::transition< i_locked_lock, Locked > > reactions;
};

struct Locked : boost::statechart::simple_state<Locked, Container::orthogonal<1> >
{
    void doo(const i_do_lock&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_lock, Locked, &Locked::doo >,
                              boost::statechart::transition< e_unlocked, Unlocked > > reactions;
};

struct UnlockedStatus : boost::statechart::simple_state<UnlockedStatus, Container::orthogonal<2> >
{
    void doo(const i_do_lockStatus&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_lockStatus, UnlockedStatus, &UnlockedStatus::doo >,
                              boost::statechart::transition< e_lockedStatus, LockedStatus > > reactions;
};

struct LockedStatus : boost::statechart::simple_state<LockedStatus, Container::orthogonal<2> >
{
    void doo(const i_do_lockStatus&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_lockStatus, LockedStatus, &LockedStatus::doo >,
                              boost::statechart::transition< e_unlockedStatus, UnlockedStatus > > reactions;
};

struct Unloaded : boost::statechart::simple_state<Unloaded, Container::orthogonal<3> >
{
    void doo(const i_do_load&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_load, Unloaded, &Unloaded::doo >,
                              boost::statechart::transition< e_loaded, Loaded > > reactions;
};

struct Loaded : boost::statechart::simple_state<Loaded, Container::orthogonal<3> >
{
    void doo(const i_do_load&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_load, Loaded, &Loaded::doo >,
                              boost::statechart::transition< e_unloaded, Unloaded > > reactions;
};

struct LessThanDesiredJawPosition : boost::statechart::simple_state<LessThanDesiredJawPosition, Container::orthogonal<4> >
{
    void doo(const i_do_position&);
 
    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_position, LessThanDesiredJawPosition, &LessThanDesiredJawPosition::doo >,
                              boost::statechart::transition< i_atDesiredJawPosition_position, AtDesiredJawPosition >,
                              boost::statechart::transition< e_moreThanDesiredJawPosition, MoreThanDesiredJawPosition > > reactions;
};

struct AtDesiredJawPosition : boost::statechart::simple_state<AtDesiredJawPosition, Container::orthogonal<4> >
{
    void doo(const i_do_position&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_position, AtDesiredJawPosition, &AtDesiredJawPosition::doo >,
                              boost::statechart::transition< e_lessThanDesiredJawPosition, LessThanDesiredJawPosition >,
                              boost::statechart::transition< e_moreThanDesiredJawPosition, MoreThanDesiredJawPosition > > reactions;
};

struct MoreThanDesiredJawPosition : boost::statechart::simple_state<MoreThanDesiredJawPosition, Container::orthogonal<4> >
{
    void doo(const i_do_position&);
 
    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_position, MoreThanDesiredJawPosition, &MoreThanDesiredJawPosition::doo >,
                              boost::statechart::transition< e_lessThanDesiredJawPosition, LessThanDesiredJawPosition >,
                              boost::statechart::transition< i_atDesiredJawPosition_position, AtDesiredJawPosition > > reactions;
};

struct NotStalled : boost::statechart::simple_state<NotStalled, Container::orthogonal<5> >
{
    void doo(const i_do_stall&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_stall, NotStalled, &NotStalled::doo >,
                              boost::statechart::transition< i_stalled_stall, Stalled > > reactions;
};

struct Stalled : boost::statechart::simple_state<Stalled, Container::orthogonal<5> >
{
    void doo(const i_do_stall&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_stall, Stalled, &Stalled::doo >,
                              boost::statechart::transition< e_notStalled, NotStalled > > reactions;
};

struct Unready : boost::statechart::simple_state<Unready, Container::orthogonal<6> >
{
    void doo(const i_do_status&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_status, Unready, &Unready::doo >,
                              boost::statechart::transition< i_ready_status, Ready > > reactions;
};

struct Ready : boost::statechart::simple_state<Ready, Container::orthogonal<6> >
{
    void doo(const i_do_status&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_status, Ready, &Ready::doo >,
                              boost::statechart::transition< i_unready_status, Unready > > reactions;
};

struct NotFaulted : boost::statechart::simple_state<NotFaulted, Container::orthogonal<7> >
{
    void doo(const i_do_fault&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_fault, NotFaulted, &NotFaulted::doo >,
                              boost::statechart::transition< i_faulted_fault, Faulted > > reactions;
};

struct Faulted : boost::statechart::simple_state<Faulted, Container::orthogonal<7> >
{
    void doo(const i_do_fault&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_fault, Faulted, &Faulted::doo >,
                              boost::statechart::transition< e_notFaulted, NotFaulted > > reactions;
};

struct Still : boost::statechart::simple_state<Still, Container::orthogonal<8> >
{
    void doo(const i_do_moving&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_moving, Still, &Still::doo >,
                              boost::statechart::transition< e_moving, Moving > > reactions;
};

struct Moving : boost::statechart::simple_state<Moving, Container::orthogonal<8> >
{
    void doo(const i_do_moving&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_moving, Moving, &Moving::doo >,
                              boost::statechart::transition< i_still_moving, Still > > reactions;
};

struct NotOpen : boost::statechart::simple_state<NotOpen, Container::orthogonal<9> >
{
    void doo(const i_do_open&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_open, NotOpen, &NotOpen::doo >,
                              boost::statechart::transition< i_isOpen_open, Open > > reactions;
};

struct Open : boost::statechart::simple_state<Open, Container::orthogonal<9> >
{
    void doo(const i_do_open&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_open, Open, &Open::doo >,
                              boost::statechart::transition< e_isNotOpen, NotOpen > > reactions;
};

struct LessThanForceThreshold : boost::statechart::state<LessThanForceThreshold, Container::orthogonal<10> >
{
    LessThanForceThreshold(my_context ctx);
    void doo(const i_do_force&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_force, LessThanForceThreshold, &LessThanForceThreshold::doo >,
                              boost::statechart::transition< e_moreThanForceThreshold, MoreThanForceThreshold > > reactions;
};

struct MoreThanForceThreshold : boost::statechart::state<MoreThanForceThreshold, Container::orthogonal<10> >
{
    MoreThanForceThreshold(my_context ctx);
    void doo(const i_do_force&);

    typedef boost::mpl::list< boost::statechart::in_state_reaction< i_do_force, MoreThanForceThreshold, &MoreThanForceThreshold::doo >,
                              boost::statechart::transition< e_lessThanForceThreshold, LessThanForceThreshold > > reactions;
};

#endif
