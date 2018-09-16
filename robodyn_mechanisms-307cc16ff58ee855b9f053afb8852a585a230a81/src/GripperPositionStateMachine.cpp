#include "nasa_robodyn_mechanisms_core/GripperPositionStateMachine.h"

using namespace log4cpp;

GripperPosition::GripperPosition()
    : mechanism(""),
      currentMax(0.0),
      currentLimitName(""),
      currentLimit(0.0),
      measuredCurrentName(""),
      measuredCurrent(0.0),
      jawOpenPosition(0.0),
      jawPositionNoise(0.0),
      encoderOpenVelocity(0.0),
      encoderCloseVelocity(0.0),
      encoderClosedPosition(0.0),
      encoderPositionNoise(0.0),
      encoderVelocityNoise(0.0),
      currentNoise(0.0),
      weakForce(0.0),
      strongForce(0.0),
      desiredForce(0.0),
      jawPositionForceThreshold(0.0),
      initializationStateName("")
{
    gripperKinematics.reset(new GripperKinematics());
}

void Container::doo(const e_do&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Container::do() - post(i_do_*)";
    post_event(i_do_fault());
    post_event(i_do_status());
    post_event(i_do_load());
    post_event(i_do_lock());
    post_event(i_do_lockStatus());
    post_event(i_do_position());
    post_event(i_do_open());
    post_event(i_do_force());
    post_event(i_do_stall());
    post_event(i_do_moving());
    post_event(i_do_active());
    post_event(i_do_movement());
}

void Container::ready(const e_ready&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::ready() - post(i_ready_*)";
    post_event(i_ready_command());
    post_event(i_ready_status());
}

void Container::unready(const e_unready&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::unready() - post(i_unready_*)";
    post_event(i_unready_command());
    post_event(i_unready_status());
}

void Container::locked(const e_locked&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::locked() - post(i_locked_*)";
    post_event(i_locked_action());
    post_event(i_locked_lock());
}

void Container::atDesiredJawPosition(const e_atDesiredJawPosition&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::atDesiredJawPosition() - post(i_atDesiredJawPosition_*)";
    post_event(i_atDesiredJawPosition_action());
    post_event(i_atDesiredJawPosition_position());
}

void Container::stalled(const e_stalled&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::stalled() - post(i_stalled_*)";
    post_event(i_stalled_action());
    post_event(i_stalled_stall());
}

void Container::faulted(const e_faulted&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::faulted() - post(i_faulted_*)";
    post_event(i_faulted_command());
    post_event(i_faulted_fault());
}

void Container::still(const e_still&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::still() - post(i_still_*)";
    post_event(i_still_action());
    post_event(i_still_moving());
}

void Container::isOpen(const e_isOpen&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Container::isOpen() - post(i_isOpen_*)";
    post_event(i_isOpen_movement());
    post_event(i_isOpen_action());
    post_event(i_isOpen_open());
}

void Container::setParameters(const c_setParameters& e)
{
    context<GripperPosition>().setPoint = e.sp;
}

Inactive::Inactive(my_context ctx) 
    : my_base(ctx)
{
    //! tare encoder position
    if (not context<GripperPosition>().encoderState.position.empty())
    {
        context<GripperPosition>().jointCommand.desiredPosition.assign(1,context<GripperPosition>().encoderState.position[0]);
    }
    if (not context<GripperPosition>().sendCommand.empty())
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Inactive::entry() - command(stop)";
        context<GripperPosition>().sendCommand(context<GripperPosition>().jointCommand);
    }
}

boost::statechart::result Inactive::react(const i_ready_command&)
{
    //! If NotFaulted is active
    if (state_downcast< const NotFaulted * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Inactive::react() - transit(Active)";
        return transit<Active>();
    }

    return discard_event();
}

Active::Active(my_context ctx)
    : my_base(ctx)
{
    desiredCurrent = 0;
    //! to help avoid "stall" conditions
    //outputCurrent = context<GripperPosition>().currentNoise;
    outputCurrent = context<GripperPosition>().getFloat(context<GripperPosition>().currentLimitName);
}

void Active::doo(const i_do_active&)
{
    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        //! use GK to find kinematic current based on single desired force value
        double averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;

        if (context<GripperPosition>().getUInt16(context<GripperPosition>().initializationStateName) > 0)
        {
            desiredCurrent = context<GripperPosition>().gripperKinematics->getMotorCurrentLimit(context<GripperPosition>().encoderState.position[0], averageJawPosition, context<GripperPosition>().desiredForce);
        }
        else
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Active::doo() - Gripper not initialized yet. desiredCurrent = currentMax";
            desiredCurrent = context<GripperPosition>().currentMax;
        }

        //! check if at desired jaw position and acting to determine whether to pass on kinematic current or max current as desired current
        // if ( (state_downcast< const AtDesiredJawPosition * >() != 0) || (state_downcast< const LessThanDesiredJawPosition * >() != 0))
        // {
        //     desiredCurrent = context<GripperPosition>().currentMax;
        // }

        //! sawtooth abs filter desired current and 0-then-output (outputCurrentFilter)
        outputCurrent = context<GripperPosition>().outputCurrentFilter.filter(outputCurrent, desiredCurrent);

        //! ceiling limit filtered current (currentMax)
        if (outputCurrent > context<GripperPosition>().currentMax)
        {
            outputCurrent = context<GripperPosition>().currentMax;
        }

        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachineDebug") << Priority::DEBUG << "encPos: " << context<GripperPosition>().encoderState.position[0] << " jawPos: " << averageJawPosition << " desiredCurrent: " << desiredCurrent << " outputCurrent: " << outputCurrent << " currentMax: " << context<GripperPosition>().currentMax;

        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Active::do() - setCurrentLimit()";
        context<GripperPosition>().setFloat(context<GripperPosition>().currentLimitName, outputCurrent);

        //! duty cycle limiting
        uint16_t previousDutyCycleLimit = context<GripperPosition>().getUInt16(context<GripperPosition>().dutyCycleLimitName);
        uint16_t newDutyCycleLimit = context<GripperPosition>().dutyCycleLimitFilter.filter(previousDutyCycleLimit, context<GripperPosition>().dutyCycleLimit);
        if (newDutyCycleLimit > context<GripperPosition>().dutyCycleMax)
        {
            newDutyCycleLimit = context<GripperPosition>().dutyCycleMax;
        }
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Active::do() - setDutyCycleLimit()";
        context<GripperPosition>().setUInt16(context<GripperPosition>().dutyCycleLimitName, newDutyCycleLimit);
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Active::do() - Missing position(s).";
    }

    //return discard_event();
}

void Active::set(const c_set&)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::PREEMPTING;
    context<GripperPosition>().goalStatus.text = "received new command, set";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Active::set() - post(e_stop, e_listen, e_set)";
    post_event(e_stop());
    post_event(e_listen());
    post_event(e_set());
}

void Active::lock(const c_lock&)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::PREEMPTING;
    context<GripperPosition>().goalStatus.text = "received new command, lock";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Active::lock() - post(e_stop, e_listen, e_lock)";
    post_event(e_stop());
    post_event(e_listen());
    post_event(e_lock());
}

void Active::release(const c_release&)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::PREEMPTING;
    context<GripperPosition>().goalStatus.text = "received new command, release";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Active::release() - post(e_stop, e_listen, e_release)";
    post_event(e_stop());
    post_event(e_listen());
    post_event(e_release());
}

void Active::cancel(const c_cancel&)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::PREEMPTED;
    context<GripperPosition>().goalStatus.text = "cancelled";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Active::cancel() - post(e_stop, e_listen)";
    post_event(e_stop());
    post_event(e_listen());
}

Stopped::Stopped(my_context ctx)
    : my_base(ctx)
{
    //! tare encoder position
    if (not context<GripperPosition>().encoderState.position.empty())
    {
        context<GripperPosition>().jointCommand.desiredPositionVelocityLimit.assign(1, 0);
    }
    if (not context<GripperPosition>().sendCommand.empty())
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Stopped::entry() - command(stop)";
        context<GripperPosition>().sendCommand(context<GripperPosition>().jointCommand);
    }
}

void Opening::doo(const i_do_movement&)
{
    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        context<GripperPosition>().jointCommand.desiredPositionVelocityLimit.assign(1, context<GripperPosition>().encoderOpenVelocity);
        if (not context<GripperPosition>().sendCommand.empty())
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Opening::do() - command(open)";
            context<GripperPosition>().sendCommand(context<GripperPosition>().jointCommand);
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Opening::do() - Missing position(s).";
    }
}

void Closing::doo(const i_do_movement&)
{
    //! send command
    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        context<GripperPosition>().jointCommand.desiredPositionVelocityLimit.assign(1, context<GripperPosition>().encoderCloseVelocity);
        if (not context<GripperPosition>().sendCommand.empty())
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Closing::do() - command(close)";
            context<GripperPosition>().sendCommand(context<GripperPosition>().jointCommand);
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Closing::do() - Missing position(s).";
    }

    //! @todo removing this for now, until we have a better idea of what initialization looks like.  might need to drive against hardstop without auto-stopping.
    //! check for closed
    // if (not context<GripperPosition>().encoderState.position.empty())
    // {
    //     if ( std::abs(context<GripperPosition>().encoderState.position[0] - context<GripperPosition>().encoderClosedPosition) <= context<GripperPosition>().encoderPositionNoise )
    //     {
    //         RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Closing::do() - post(e_isClosed)";
    //         post_event(e_isClosed());
    //     }
    // }
}

Listening::Listening(my_context ctx)
    : my_base(ctx)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Listening::entry() - post(e_stop)";
    post_event(e_stop());
}

OpenWide::OpenWide(my_context ctx)
    : my_base(ctx)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;
    context<GripperPosition>().goalStatus.text = "opening wide";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    //! If NotOpen is active
    if (state_downcast< const NotOpen * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] OpenWide::entry() - post(e_open)";
        post_event(e_open());
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] OpenWide::entry() - post(e_next)";
        post_event(e_next());
    }
}

boost::statechart::result OpenWide::react(const i_stalled_action&)
{
    //! If Open is active
    if (state_downcast< const Open * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] OpenWide - transit(JawPositionCheck)";
        return transit<JawPositionCheck>();
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] OpenWide - Failed to complete.";

        context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ABORTED;
        context<GripperPosition>().goalStatus.text = "encoder still and jaws not open";
        if (not context<GripperPosition>().sendGoalStatus.empty())
        {
            context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
        }

        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] OpenWide::react() - transit(Listening)";
        return transit<Listening>();
    }

    return discard_event();
}

JawPositionCheck::JawPositionCheck(my_context ctx)
    : my_base(ctx)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;
    context<GripperPosition>().goalStatus.text = "checking jaw position";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    //! If MoreThanDesiredJawPosition is active
    if (state_downcast< const MoreThanDesiredJawPosition * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] JawPositionCheck::entry() - post(e_close)";
        post_event(e_close());
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] JawPositionCheck::entry() - post(e_next)";
        post_event(e_next());
    }
}

boost::statechart::result JawPositionCheck::react(const i_stalled_action&)
{
    //! If AtDesiredJawPosition is NOT active
    if (state_downcast< const AtDesiredJawPosition * >() == 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] JawPositionCheck - Failed to complete.";

        context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ABORTED;
        context<GripperPosition>().goalStatus.text = "encoder still and not at desired jaw position";
        if (not context<GripperPosition>().sendGoalStatus.empty())
        {
            context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
        }

        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] JawPositionCheck::react() - transit(Listening)";
        return transit<Listening>();
    }

    return discard_event();
}

Close::Close(my_context ctx)
    : my_base(ctx)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;
    context<GripperPosition>().goalStatus.text = "closing";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    //! If MoreThanDesiredJawPositionOrLess is active
    if (state_downcast< const MoreThanDesiredJawPosition * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Close::entry() - post(e_close)";
        post_event(e_close());
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Close::entry() - post(e_next)";
        post_event(e_next());
    }
}

boost::statechart::result Close::react(const i_stalled_action&)
{
    //! If MoreThanDesiredJawPositionOrLess is active
    if (state_downcast< const MoreThanDesiredJawPosition * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Close - Failed to complete.";

        context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ABORTED;
        context<GripperPosition>().goalStatus.text = "encoder still and not at desired jaw position or less";
        if (not context<GripperPosition>().sendGoalStatus.empty())
        {
            context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
        }

        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Close::react() - transit(Listening)";
        return transit<Listening>();
    }

    return discard_event();
}

LockCheck::LockCheck(my_context ctx)
    : my_base(ctx)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;
    context<GripperPosition>().goalStatus.text = "checking for over center";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    post_event(e_close());
}

boost::statechart::result LockCheck::react(const i_stalled_action&)
{
    //! If Unlocked is active
    if (state_downcast< const Unlocked * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] LockCheck - Failed to complete.";

        context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ABORTED;
        context<GripperPosition>().goalStatus.text = "encoder still and not over center";
        if (not context<GripperPosition>().sendGoalStatus.empty())
        {
            context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
        }

        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] LockCheck::react() - transit(Listening)";
        return transit<Listening>();
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] LockCheck::react() - post(e_next)";
        post_event(e_next());
    }

    return discard_event();
}

OpenUntilStall::OpenUntilStall(my_context ctx)
    : my_base(ctx)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;
    context<GripperPosition>().goalStatus.text = "opening until stall detected";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    //! If NotOpen is active
    if (state_downcast< const NotOpen * >() != 0)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] OpenUntilStall::entry() - post(e_open)";
        post_event(e_open());
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] OpenUntilStall::entry() - post(e_next)";
        post_event(e_next());
    }
}

Done::Done(my_context ctx)
    : my_base (ctx)
{
    context<GripperPosition>().goalStatus.status = actionlib_msgs::GoalStatus::SUCCEEDED;
    context<GripperPosition>().goalStatus.text = "complete";
    if (not context<GripperPosition>().sendGoalStatus.empty())
    {
        context<GripperPosition>().sendGoalStatus(context<GripperPosition>().goalStatus, true);
    }

    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Done::entry() - post(e_stop, e_done)";
    post_event(e_stop());
    post_event(e_done());
}

void Unlocked::doo(const i_do_lock&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Unlocked::do()";

    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;

        if (context<GripperPosition>().getUInt16(context<GripperPosition>().initializationStateName) > 0)
        {
            if (context<GripperPosition>().gripperKinematics->isOverCenter(context<GripperPosition>().encoderState.position[0], averageJawPosition))
            {
                RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Unlocked::doo() - post(e_locked)";
                post_event(e_locked());
            }
        }
        else
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Unlocked::doo() - Gripper not initialized yet.";
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Unlocked::do() - Missing position(s).";
    }
}

void Locked::doo(const i_do_lock&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Locked::do()";

    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (context<GripperPosition>().getUInt16(context<GripperPosition>().initializationStateName) > 0)
        {
            if (not context<GripperPosition>().gripperKinematics->isOverCenter(context<GripperPosition>().encoderState.position[0], averageJawPosition))
            {
                RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Locked::doo() - post(e_unlocked)";
                post_event(e_unlocked());
            }
        }
        else
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Locked::doo() - Gripper not initialized yet.";
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Locked::do() - Missing position(s).";
    }
}

void UnlockedStatus::doo(const i_do_lockStatus&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] UnlockedStatus::do()";

    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (context<GripperPosition>().getUInt16(context<GripperPosition>().initializationStateName) > 0)
        {
            if (context<GripperPosition>().gripperKinematics->isOverCenterStatus(context<GripperPosition>().encoderState.position[0], averageJawPosition))
            {
                RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] UnlockedStatus::doo() - post(e_lockedStatus)";
                post_event(e_lockedStatus());
            }
        }
        else
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] UnlockedStatus::doo() - Gripper not initialized yet.";
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] UnlockedStatus::do() - Missing position(s).";
    }
}

void LockedStatus::doo(const i_do_lockStatus&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] LockedStatus::do()";

    if ((not context<GripperPosition>().encoderState.position.empty()) && (not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (context<GripperPosition>().getUInt16(context<GripperPosition>().initializationStateName) > 0)
        {
            if (not context<GripperPosition>().gripperKinematics->isOverCenterStatus(context<GripperPosition>().encoderState.position[0], averageJawPosition))
            {
                RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] LockedStatus::doo() - post(e_unlockedStatus)";
                post_event(e_unlockedStatus());
            }
        }
        else
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] LockedStatus::doo() - Gripper not initialized yet.";
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] LockedStatus::do() - Missing position(s).";
    }
}

void Unloaded::doo(const i_do_load&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Unloaded::do()";

    if ((not context<GripperPosition>().jawLeftState.effort.empty()) && (not context<GripperPosition>().jawRightState.effort.empty()))
    {
        float averageJawLoad = (context<GripperPosition>().jawLeftState.effort[0] + context<GripperPosition>().jawRightState.effort[0]) / 2.0;
        if (averageJawLoad >= (context<GripperPosition>().setPoint.expectedLoad - context<GripperPosition>().setPoint.expectedLoadDelta))
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Unloaded::do() - post(e_loaded)";
            post_event(e_loaded());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Unloaded::do() - Missing effort(s).";
    }
}

void Loaded::doo(const i_do_load&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Loaded::do()";

    if ((not context<GripperPosition>().jawLeftState.effort.empty()) && (not context<GripperPosition>().jawRightState.effort.empty()))
    {
        float averageJawLoad = (context<GripperPosition>().jawLeftState.effort[0] + context<GripperPosition>().jawRightState.effort[0]) / 2.0;
        if (averageJawLoad < (context<GripperPosition>().setPoint.expectedLoad - context<GripperPosition>().setPoint.expectedLoadDelta))
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Loaded::do() - post(e_unloaded)";
            post_event(e_unloaded());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Loaded::do() - Missing effort(s).";
    }
}

void LessThanDesiredJawPosition::doo(const i_do_position&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] LessThanDesiredJawPosition::do()";

    //! Make sure both jaws have state messages populated
    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        //! Compare average jaw position against set point and delta
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;

        if ( std::abs(averageJawPosition - context<GripperPosition>().setPoint.jawPosition) <= context<GripperPosition>().setPoint.jawPositionDelta )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] LessThanDesiredJawPosition::do() - post(e_atDesiredJawPosition)";
            post_event(e_atDesiredJawPosition());
        }

        if ( averageJawPosition > (context<GripperPosition>().setPoint.jawPosition + context<GripperPosition>().setPoint.jawPositionDelta) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] LessThanDesiredJawPosition::do() - post(e_moreThanDesiredJawPosition)";
            post_event(e_moreThanDesiredJawPosition());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] LessThanDesiredJawPosition::do() - Missing jaw position(s).";
    }
}

void AtDesiredJawPosition::doo(const i_do_position&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] AtDesiredJawPosition::do()";

    //! Make sure both jaws have state messages populated
    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        //! Compare average jaw position against set point and delta
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;

        if ( averageJawPosition < (context<GripperPosition>().setPoint.jawPosition - context<GripperPosition>().setPoint.jawPositionDelta) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] AtDesiredJawPosition::do() - post(e_lessThanDesiredJawPosition)";
            post_event(e_lessThanDesiredJawPosition());
        }

        if ( averageJawPosition > (context<GripperPosition>().setPoint.jawPosition + context<GripperPosition>().setPoint.jawPositionDelta) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] AtDesiredJawPosition::do() - post(e_moreThanDesiredJawPosition)";
            post_event(e_moreThanDesiredJawPosition());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] AtDesiredJawPosition::do() - Missing jaw position(s).";
    }
}

void MoreThanDesiredJawPosition::doo(const i_do_position&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] MoreThanDesiredJawPosition::do()";

    //! Make sure both jaws have state messages populated
    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        //! Compare average jaw position against set point and delta
        float averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;

        if ( std::abs(averageJawPosition - context<GripperPosition>().setPoint.jawPosition) <= context<GripperPosition>().setPoint.jawPositionDelta )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] MoreThanDesiredJawPosition::do() - post(e_atDesiredJawPosition)";
            post_event(e_atDesiredJawPosition());
        }

        if ( averageJawPosition < (context<GripperPosition>().setPoint.jawPosition - context<GripperPosition>().setPoint.jawPositionDelta) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] MoreThanDesiredJawPosition::do() - post(e_lessThanDesiredJawPosition)";
            post_event(e_lessThanDesiredJawPosition());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] MoreThanDesiredJawPosition::do() - Missing jaw position(s).";
    }
}

void NotStalled::doo(const i_do_stall&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] NotStalled::do()";

    //! if at current limit
    if (std::abs(context<GripperPosition>().measuredCurrent) >= (context<GripperPosition>().currentLimit - context<GripperPosition>().currentNoise))
    {
        //! and not moving
        if (state_downcast< const Still * >() != 0)
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] NotStalled::do() - post(e_stalled)";
            post_event(e_stalled());
        }
    }
}

void Stalled::doo(const i_do_stall&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Stalled::do()";

    if (std::abs(context<GripperPosition>().measuredCurrent) < (context<GripperPosition>().currentLimit + context<GripperPosition>().currentNoise))
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Stalled::do() - post(e_notStalled)";
        post_event(e_notStalled());
    }
}

void Unready::doo(const i_do_status&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Unready::do()";

    if (context<GripperPosition>().jointControlData.controlMode.state == nasa_r2_common_msgs::JointControlMode::DRIVE)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Unready::do() - post(e_ready)";
        post_event(e_ready());
    }
}

void Ready::doo(const i_do_status&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Ready::do()";

    if (context<GripperPosition>().jointControlData.controlMode.state != nasa_r2_common_msgs::JointControlMode::DRIVE)
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Ready::do() - post(e_unready)";
        post_event(e_unready());
    }
}

void NotFaulted::doo(const i_do_fault&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] NotFaulted::do()";
    //! @todo calculate if actually faulted
    // if (true)
    // {
    //     post_event(e_faulted());
    // }
}

void Faulted::doo(const i_do_fault&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Faulted::do()";
    //! @todo calculate if actually not faulted
    // if (true)
    // {
    //     post_event(e_notFaulted());
    // }
}

void Still::doo(const i_do_moving&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Still::do()";

    if (not context<GripperPosition>().encoderState.velocity.empty())
    {
        if (std::abs(context<GripperPosition>().encoderState.velocity[0]) >= context<GripperPosition>().encoderVelocityNoise)
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Still::do() - post(e_moving)";
            post_event(e_moving());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Still::do() - Missing encoder velocity.";
    }
}

void Moving::doo(const i_do_moving&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Moving::do()";

    if (not context<GripperPosition>().encoderState.velocity.empty())
    {
        if (std::abs(context<GripperPosition>().encoderState.velocity[0]) < context<GripperPosition>().encoderVelocityNoise)
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Moving::do() - post(e_still)";
            post_event(e_still());
        }
    }
    else
    {
        RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::WARN << "[" << context<GripperPosition>().mechanism << "] Moving::do() - Missing encoder velocity.";
    }
}

void NotOpen::doo(const i_do_open&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] NotOpen::do()";

    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        double averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (averageJawPosition >= (context<GripperPosition>().jawOpenPosition - context<GripperPosition>().jawPositionNoise) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] NotOpen::do() - post(e_isOpen)";
            post_event(e_isOpen());
        }
    }
}

void Open::doo(const i_do_open&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] Open::do()";

    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        double averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (averageJawPosition < (context<GripperPosition>().jawOpenPosition - context<GripperPosition>().jawPositionNoise) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] Open::do() - post(e_isNotOpen)";
            post_event(e_isNotOpen());
        }
    }
}

LessThanForceThreshold::LessThanForceThreshold(my_context ctx) 
    : my_base(ctx)
{
    context<GripperPosition>().desiredForce = context<GripperPosition>().strongForce;
}

void LessThanForceThreshold::doo(const i_do_force&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] LessThanForceThreshold::do()";

    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        double averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (averageJawPosition >= (context<GripperPosition>().jawPositionForceThreshold + context<GripperPosition>().jawPositionNoise) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] LessThanForceThreshold::do() - post(e_moreThanForceThreshold)";
            post_event(e_moreThanForceThreshold());
        }
    }
}

MoreThanForceThreshold::MoreThanForceThreshold(my_context ctx) 
    : my_base(ctx)
{
    context<GripperPosition>().desiredForce = context<GripperPosition>().weakForce;
}

void MoreThanForceThreshold::doo(const i_do_force&)
{
    RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::DEBUG << "[" << context<GripperPosition>().mechanism << "] MoreThanForceThreshold::do()";

    if ((not context<GripperPosition>().jawLeftState.position.empty()) && (not context<GripperPosition>().jawRightState.position.empty()))
    {
        double averageJawPosition = (context<GripperPosition>().jawLeftState.position[0] + context<GripperPosition>().jawRightState.position[0]) / 2.0;
        
        if (averageJawPosition < (context<GripperPosition>().jawPositionForceThreshold + context<GripperPosition>().jawPositionNoise) )
        {
            RCS::Logger::getCategory("gov.nasa.robodyn.mechanisms.GripperPositionStateMachine") << Priority::INFO << "[" << context<GripperPosition>().mechanism << "] MoreThanForceThreshold::do() - post(e_lessThanForceThreshold)";
            post_event(e_lessThanForceThreshold());
        }
    }
}