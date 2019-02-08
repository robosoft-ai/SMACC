/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <ros/ros.h>
#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/fifo_scheduler.hpp>
#include <boost/statechart/asynchronous_state_machine.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include <boost/config.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/mpl/list.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>

#include <boost/mpl/list.hpp>
#include <actionlib/client/simple_action_client.h>

#include <boost/statechart/termination.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/any.hpp>
#include <boost/algorithm/string.hpp>

namespace sc = boost::statechart;

using namespace boost;

typedef sc::fifo_scheduler<> SmaccScheduler;

typedef std::allocator< void > SmaccAllocator;


template<class T>
auto optionalNodeHandle(boost::intrusive_ptr<T>& obj)
 -> decltype(  obj->nh  )
{
    return obj->nh;
}

template<class T>
auto optionalNodeHandle(T* ) -> ros::NodeHandle
{
  return ros::NodeHandle("");
}

//typedef boost::fast_pool_allocator< int > MyAllocator;

//template <typename MostDerived, typename Context, typename InnerList= mpl::list<>>
//using SmaccState = sc::state<MostDerived,Context,InnerList>;


//------------------------------------------------------------

typedef boost::statechart::processor_container<boost::statechart::fifo_scheduler<>, boost::function0<void>, std::allocator<void> >::processor_context my_context;
namespace smacc
{
  class ISmaccStateMachine;
  class ISmaccComponent;
  class ISmaccActionClient;
  class SignalDetector;

  struct IActionResult
  {
    smacc::ISmaccActionClient* client;
    actionlib::SimpleClientGoalState getResult() const;
  };

  template <typename ActionResult>
  struct EvActionResult : sc::event< EvActionResult <ActionResult> >, IActionResult
  { 
      ActionResult resultMessage;
  };

  template <typename ActionFeedback>
  struct EvActionFeedback : sc::event< EvActionFeedback <ActionFeedback>> 
  {
      smacc::ISmaccActionClient* client;
      ActionFeedback feedbackMessage;
      //boost::any feedbackMessage;
  };

  // demangles the type name to be used as a node handle path
  std::string cleanTypeName(const std::type_info& tinfo);
}



inline std::string demangleSymbol(const char* name)
{
  #if (__GNUC__ && __cplusplus && __GNUC__ >= 3)
    int         status;
    char* res = abi::__cxa_demangle(name, 0, 0, &status);
    if (res)
    {
      const std::string demangled_name(res);
      std::free(res);
      return demangled_name;
    }
    // Demangling failed, fallback to mangled name
    return std::string(name);
  #else
    return std::string(name);
  #endif
}
template <class T>
inline std::string demangledTypeName()
{
  return demangleSymbol(typeid(T).name());
}
