/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/asynchronous_state_machine.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deep_history.hpp>

#include <boost/config.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <boost/mpl/list.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/any.hpp>
#include <boost/algorithm/string.hpp>

#include <mutex>

#include <ros/ros.h>
//#include <actionlib/client/simple_action_client.h>

#include <smacc/smacc_fifo_scheduler.h>
#include <smacc/smacc_types.h>
#include <smacc/introspection/introspection.h>

typedef boost::statechart::processor_container<boost::statechart::fifo_scheduler<>, boost::function0<void>, std::allocator<void>>::processor_context my_context;
namespace smacc
{

namespace utils
{
// demangles the type name to be used as a node handle path
std::string cleanShortTypeName(const std::type_info &tinfo);
} // namespace utils

enum class SMRunMode
{
  DEBUG,
  RELEASE
};

} // namespace smacc

#include <smacc/smacc_default_events.h>
#include <smacc/smacc_transition.h>
