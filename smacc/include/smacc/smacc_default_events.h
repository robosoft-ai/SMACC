#pragma once

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>

#include <smacc/smacc_types.h>

namespace smacc
{

template <typename ActionFeedback>
struct EvActionFeedback : sc::event<EvActionFeedback<ActionFeedback>>
{
  smacc::ISmaccActionClient *client;
  ActionFeedback feedbackMessage;
  //boost::any feedbackMessage;
};

template <typename TSource, typename TObjectTag = default_object_tag>
struct EvActionResult : sc::event<EvActionResult<TSource, default_object_tag>>, IActionResult
{
  typename TSource::Result resultMessage;
};

//--------------------------------
template <typename TSource>
struct EvActionSucceeded : sc::event<EvActionResult<TSource>>, IActionResult
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

template <typename TSource>
struct EvActionAborted : sc::event<EvActionResult<TSource>>, IActionResult
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

template <typename TSource>
struct EvActionPreempted : sc::event<EvActionResult<TSource>>, IActionResult
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

template <typename TSource>
struct EvActionRejected : sc::event<EvActionResult<TSource>>, IActionResult
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
};

template <typename TSource>
struct EvLoopEnd : sc::event<EvLoopEnd<TSource>>
{
};

} // namespace smacc
