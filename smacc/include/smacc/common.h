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
#include <boost/statechart/deep_history.hpp>
#include <boost/any.hpp>
#include <boost/algorithm/string.hpp>

namespace sc = boost::statechart;

using namespace boost;

typedef sc::fifo_scheduler<> SmaccScheduler;

typedef std::allocator<void> SmaccAllocator;

template <class T>
auto optionalNodeHandle(boost::intrusive_ptr<T> &obj)
    -> decltype(obj->nh)
{
  return obj->nh;
}

template <class T>
auto optionalNodeHandle(T *) -> ros::NodeHandle
{
  return ros::NodeHandle("");
}


inline std::string demangleSymbol(const std::string &name)
{
  return demangleSymbol(name.c_str());
}

inline std::string demangleSymbol(const char *name)
{
#if (__GNUC__ && __cplusplus && __GNUC__ >= 3)
  int status;
  char *res = abi::__cxa_demangle(name, 0, 0, &status);
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

template <typename T>
inline std::string demangleSymbol()
{
  return demangleSymbol(typeid(T).name());
}

template <class T>
inline std::string demangledTypeName()
{
  return demangleSymbol(typeid(T).name());
}

//typedef boost::fast_pool_allocator< int > MyAllocator;

//template <typename MostDerived, typename Context, typename InnerList= mpl::list<>>
//using SmaccState = sc::state<MostDerived,Context,InnerList>;

//------------------------------------------------------------

typedef boost::statechart::processor_container<boost::statechart::fifo_scheduler<>, boost::function0<void>, std::allocator<void>>::processor_context my_context;
namespace smacc
{
class ISmaccStateMachine;
class ISmaccComponent;
class ISmaccClient;

class SmaccSubStateBehavior;
class ISmaccActionClient;
class ISmaccSubscriber;
class SignalDetector;
class SmaccStateMachineInfo;
class SmaccStateInfo;
class LogicUnit;

struct IActionResult
{
  smacc::ISmaccActionClient *client;
  actionlib::SimpleClientGoalState getResultState() const;
};

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

//-------------------------------------------------------------------------
template <typename T>
class HasEventLabel
{
private:
  typedef char YesType[1];
  typedef char NoType[2];

  template <typename C>
  static YesType &test(decltype(&C::getEventLabel));
  template <typename C>
  static NoType &test(...);

public:
  enum
  {
    value = sizeof(test<T>(0)) == sizeof(YesType)
  };
};

template <typename T>
typename std::enable_if<HasEventLabel<T>::value, void>::type
EventLabel(std::string &label)
{
  label = T::getEventLabel();
}

template <typename T>
typename std::enable_if<!HasEventLabel<T>::value, void>::type
EventLabel(std::string &label)
{
  label = "";
}
//-----------------------------------------------------------------------

template <typename T>
class HasAutomaticTransitionTag
{
private:
  typedef char YesType[1];
  typedef char NoType[2];

  template <typename C>
  static YesType &test(decltype(&C::getDefaultTransitionTag));
  template <typename C>
  static NoType &test(...);

public:
  enum
  {
    value = sizeof(test<T>(0)) == sizeof(YesType)
  };
};

template <typename T>
typename std::enable_if<HasAutomaticTransitionTag<T>::value, void>::type
automaticTransitionTag(std::string &transition_name)
{
  transition_name = T::getDefaultTransitionTag();
}

template <typename T>
typename std::enable_if<!HasAutomaticTransitionTag<T>::value, void>::type
automaticTransitionTag(std::string &transition_name)
{
  transition_name = "";
}

//-------------------------------------------------
template <typename T>
class HasAutomaticTransitionType
{
private:
  typedef char YesType[1];
  typedef char NoType[2];

  template <typename C>
  static YesType &test(decltype(&C::getDefaultTransitionType));
  template <typename C>
  static NoType &test(...);

public:
  enum
  {
    value = sizeof(test<T>(0)) == sizeof(YesType)
  };
};

template <typename T>
typename std::enable_if<HasAutomaticTransitionType<T>::value, void>::type
automaticTransitionType(std::string &transition_type)
{
  transition_type = T::getDefaultTransitionType();
}

template <typename T>
typename std::enable_if<!HasAutomaticTransitionType<T>::value, void>::type
automaticTransitionType(std::string &transition_type)
{
  transition_type = demangledTypeName<DEFAULT>();
}
//--------------------------------
template <typename ActionFeedback>
struct EvActionFeedback : sc::event<EvActionFeedback<ActionFeedback>>
{
  smacc::ISmaccActionClient *client;
  ActionFeedback feedbackMessage;
  //boost::any feedbackMessage;
};

// demangles the type name to be used as a node handle path
std::string cleanShortTypeName(const std::type_info &tinfo);

enum class SMRunMode
{
  DEBUG,
  RELEASE
};

template <typename StateMachineType>
void run();
} // namespace smacc

#include <boost/mpl/for_each.hpp>

namespace smacc
{
// there are many ways to implement this, for instance adding static methods to the types
typedef mpl::list<SUCCESS, ABORT, PREEMPT, CONTINUELOOP, ENDLOOP> DEFAULT_TRANSITION_TYPES;


//--------------------------------

template <typename T>
struct type_
{
    using type = T;
};

//---------------------------------------------
template <typename T>
struct add_type_wrapper
{
    using type = type_<T>;
};


template <typename TTransition>
struct CheckType
{
  CheckType(std::string* transitionTypeName)
  {
    this->transitionTypeName = transitionTypeName;

  }

  std::string* transitionTypeName;
  template <typename T>
  void operator()(T)
  {
    //ROS_INFO_STREAM("comparing.."<< demangleSymbol<T>() <<" vs " << demangleSymbol<TTransition>() );
    if (std::is_base_of<T, TTransition>::value || std::is_same<T, TTransition>::value)
    {
      *(this->transitionTypeName) = demangledTypeName<T>();
      //ROS_INFO("YESS!");
    }
  }
};

template <typename TTransition>
static std::string getTransitionType()
{
  std::string output;
  CheckType<TTransition> op(&output);
  using boost::mpl::_1;
  using wrappedList = typename boost::mpl::transform<DEFAULT_TRANSITION_TYPES, _1>::type;

  mpl::for_each<wrappedList>(op);
  return output;
};


//-------------------------------------------------------------------------

template <typename TSource, typename TObjectTag = default_object_tag>
struct EvActionResult : sc::event<EvActionResult<TSource, default_object_tag>>, IActionResult
{
  typename TSource::Result resultMessage;
};

//--------------------------------
template <typename TSource>
struct EvActionSucceded : sc::event<EvActionResult<TSource>>, IActionResult
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



#include <smacc/transition.h>
