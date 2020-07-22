/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/introspection/introspection.h>

namespace smacc
{
// template<typename TState>
// struct HasSpecificNamedOnExit
// {
//     template<typename U, size_t (U::*)() const> struct SFINAE {};
//     template<typename U> static char Test(SFINAE<U, &U::onExit(SUCCESS)>*);
//     template<typename U> static int Test(...);
//     static const bool Has = sizeof(Test<TState>(0)) == sizeof(char);
// };

template<typename T, typename TransitionTagName>
class HasSpecificNamedOnExit
{
    template <typename U, void (U::*)(TransitionTagName)> struct Check;
    template <typename U> static char func(Check<U, &U::onExit> *);
    template <typename U> static int func(...);
  public:
    typedef HasSpecificNamedOnExit type;
    enum { value = sizeof(func<T>(0)) == sizeof(char) };
};

template<typename TState, typename TTransitionTagName>
void specificNamedOnExit(TState& st, TTransitionTagName tn, std::true_type)
{
    st.onExit(tn);
}

template<typename TState, typename TTransitionTagName>
void specificNamedOnExit(TState&, TTransitionTagName tn, std::false_type)
{
}

template<typename TState, typename TTransitionTagName>
void specificNamedOnExit(TState& m, TTransitionTagName tn)
{
    specificNamedOnExit(m, tn,
        std::integral_constant<bool, HasSpecificNamedOnExit<TState, TTransitionTagName>::value>());
}

//////////////////////////////////////////////////////////////////////////////
template <class Event,
          class Destination,
          typename Tag,
          class TransitionContext,
          void (TransitionContext::*pTransitionAction)(const Event &)>
class Transition
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
      ROS_DEBUG("[Smacc Transition] REACT WITHOUT ACTION");
      typedef smacc::Transition<Event, Destination, Tag, TransitionContext, pTransitionAction> Transtype;
      TRANSITION_TAG mock;
      specificNamedOnExit(stt, mock);

      stt.template notifyTransition<Transtype>();
      return stt.template transit<Destination>();
    }

    static boost::statechart::result react_with_action(State &stt, const Event &evt)
    {
      ROS_DEBUG("[Smacc Transition] REACT WITH ACTION AND EVENT");
      typedef smacc::Transition<Event, Destination, Tag, TransitionContext, pTransitionAction> Transtype;
      TRANSITION_TAG mock;
      specificNamedOnExit(stt, mock);
      stt.template notifyTransition<Transtype>();
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