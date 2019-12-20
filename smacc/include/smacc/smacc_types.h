#pragma once

#include <boost/statechart/transition.hpp>

namespace smacc
{
class ISmaccState;
class ISmaccStateMachine;
class ISmaccClient;
class ISmaccUpdatable;
class ISmaccComponent;
class SmaccClientBehavior;
class SignalDetector;

class ISmaccActionClient;
class ISmaccSubscriber;
class SmaccStateMachineInfo;
class SmaccStateInfo;
class LogicUnit;

// ----TAGS FOR TRANSITIONS -----

// all transitions are by default labeled with this structname
struct default_object_tag
{
};

// you can also use these other labels in order to have
// a better code readability and also to improve the visual representation
// in the viewer
struct DEFAULT
{
};

struct ABORT
{
};
struct SUCCESS
{
};
struct PREEMPT
{
};
struct REJECT
{
};

struct CONTINUELOOP
{
};
struct ENDLOOP
{
};

struct default_transition_name : smacc::SUCCESS
{
};

template <class Event,
          class Destination,
          typename Tag = default_transition_name,
          class TransitionContext = boost::statechart::detail::no_context<Event>,
          void (TransitionContext::*pTransitionAction)(const Event &) =
              &boost::statechart::detail::no_context<Event>::no_function>
class transition;

} // namespace smacc
