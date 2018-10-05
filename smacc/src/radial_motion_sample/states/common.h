
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
#include <boost/statechart/termination.hpp>

namespace sc = boost::statechart;

using namespace boost;

typedef sc::fifo_scheduler<> SmaccScheduler;

typedef std::allocator< void > SmaccAllocator;
//typedef boost::fast_pool_allocator< int > MyAllocator;


template <typename MostDerived, typename Context, typename InnerList= mpl::list<>>
using SmaccState = sc::state<MostDerived,Context,InnerList>;

struct EvActionClientSuccess : sc::event< EvActionClientSuccess > 
{
    std::string target;
};

struct EvStateFinished : sc::event< EvStateFinished > 
{
};

// ------------------------------------------------------------------------
namespace NavigateToRadialStart
{
    struct State;
};

struct RotateDegress;

/// State Machine
struct SmaccStateMachine : sc::asynchronous_state_machine<SmaccStateMachine, NavigateToRadialStart::State, SmaccScheduler, SmaccAllocator >
{
    public:
      SmaccStateMachine( my_context ctx, std::shared_ptr<boost::signals2::signal<void (std::string actionClientRequestInfo) >> requestSignal);
      
      virtual ~SmaccStateMachine( )
      {
          ROS_INFO("Finishing State Machine");
      }

    /// used by the states when an action client request is launched
    void registerActionClientRequest(std::string actionClientRequestInfo)
    {
        ROS_INFO("Registering action client request: %s", actionClientRequestInfo.c_str());
        (*onNewActionClientRequest)(actionClientRequestInfo);
    }
    
    //event to notify to the signaldetection thread that a request has been created
    std::shared_ptr<boost::signals2::signal<void (std::string actionClientRequestInfo) >> onNewActionClientRequest;

    private:
        // This function is defined in the Player.cpp
        virtual void initiate_impl() override;
};

