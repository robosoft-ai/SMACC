#pragma once

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>

#include <smacc/smacc_types.h>

namespace smacc
{

using namespace smacc::introspection;
template <typename ActionFeedback, typename TObjectTag>
struct EvActionFeedback : sc::event<EvActionFeedback<ActionFeedback, TObjectTag>>
{
  smacc::client_bases::ISmaccActionClient *client;
  ActionFeedback feedbackMessage;
  //boost::any feedbackMessage;
};

template <typename TSource, typename TObjectTag>
struct EvActionResult : sc::event<EvActionResult<TSource, default_object_tag>>, IActionResult
{
  typename TSource::Result resultMessage;
};

//--------------------------------
template <typename TSource, typename TObjectTag>
struct EvActionSucceeded : sc::event<EvActionSucceeded<TSource, TObjectTag>>, IActionResult
{
  typename TSource::Result resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag()
  {
    return demangledTypeName<SUCCESS>();
  }

  static std::string getDefaultTransitionType()
  {
    return demangledTypeName<SUCCESS>();
  }
};

template <typename TSource, typename TObjectTag>
struct EvActionAborted : sc::event<EvActionAborted<TSource, TObjectTag>>, IActionResult
{
  typename TSource::Result resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag()
  {
    return demangledTypeName<ABORT>();
  }

  static std::string getDefaultTransitionType()
  {
    return demangledTypeName<ABORT>();
  }
};

template <typename TSource, typename TObjectTag>
struct EvActionPreempted : sc::event<EvActionPreempted<TSource, TObjectTag>>, IActionResult
{
  typename TSource::Result resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag()
  {
    return demangledTypeName<PREEMPT>();
  }

  static std::string getDefaultTransitionType()
  {
    return demangledTypeName<PREEMPT>();
  }
};

template <typename TSource, typename TObjectTag>
struct EvActionRejected : sc::event<EvActionRejected<TSource, TObjectTag>>, IActionResult
{
  typename TSource::Result resultMessage;

  static std::string getEventLabel()
  {
    // show ros message type
    std::string label;
    EventLabel<TSource>(label);
    return label;
  }

  static std::string getDefaultTransitionTag()
  {
    return demangledTypeName<REJECT>();
  }

  static std::string getDefaultTransitionType()
  {
    return demangledTypeName<REJECT>();
  }
};

template <typename StateType>
struct EvStateFinish : sc::event<EvStateFinish<StateType>>
{
  StateType *state;
};

template <typename TSource>
struct EvLoopContinue : sc::event<EvLoopContinue<TSource>>
{
  static std::string getDefaultTransitionTag()
  {
    return demangledTypeName<CONTINUELOOP>();
  }

  static std::string getDefaultTransitionType()
  {
    return demangledTypeName<CONTINUELOOP>();
  }
};

template <typename TSource>
struct EvLoopEnd : sc::event<EvLoopEnd<TSource>>
{
  static std::string getDefaultTransitionTag()
  {
    return demangledTypeName<ENDLOOP>();
  }

  static std::string getDefaultTransitionType()
  {
    return demangledTypeName<ENDLOOP>();
  }
};

} // namespace smacc
