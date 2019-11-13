#pragma once

#include <smacc/common.h>
#include <boost/statechart/transition.hpp>

namespace smacc
{

struct default_transition_name : smacc::SUCCESS
{
};

//////////////////////////////////////////////////////////////////////////////
template <class Event,
          class Destination,
          typename Tag = default_transition_name,
          class TransitionContext = boost::statechart::detail::no_context<Event>,
          void (TransitionContext::*pTransitionAction)(const Event &) =
              &boost::statechart::detail::no_context<Event>::no_function>
class transition 
{
public:
  typedef Tag TRANSITION_TAG;

private:
  //////////////////////////////////////////////////////////////////////////
  template <class State>
  struct reactions
  {
    static boost::statechart::result react_without_action(State &stt)
    {
      ROS_ERROR("REACT WITHOUT ACTION");
      typedef smacc::transition<Event, Destination, Tag, TransitionContext, pTransitionAction> Transtype;

      stt.template notifyTransition<Transtype>();
      return stt.template transit<Destination>();
    }

    static boost::statechart::result react_with_action(State &stt, const Event &evt)
    {
      ROS_ERROR("REACT WITH ACTION AND EVENT");
      typedef smacc::transition<Event, Destination, Tag, TransitionContext, pTransitionAction> Transtype;
      stt.template notifyTransition<Transtype> ();
      return stt.template transit<Destination>(pTransitionAction, evt);
    }
  };

public:
  //////////////////////////////////////////////////////////////////////////
  // The following declarations should be private.
  // They are only public because many compilers lack template friends.
  //////////////////////////////////////////////////////////////////////////
  template <class State, class EventBase, class IdType>
  static boost::statechart::detail::reaction_result react(
      State &stt, const EventBase &evt, const IdType &eventType)
  {
    typedef boost::statechart::detail::reaction_dispatcher<
        reactions<State>, State, EventBase, Event, TransitionContext, IdType>
        dispatcher;
    return dispatcher::react(stt, evt, eventType);
  }

};

} // namespace smacc