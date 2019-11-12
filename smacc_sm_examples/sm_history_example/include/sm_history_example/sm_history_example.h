//////////////////////////////////////////////////////////////////////////////
// Copyright 2005-2006 Andreas Huber Doenni
// Distributed under the Boost Software License, Version 1.0. (See accompany-
// ing file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//////////////////////////////////////////////////////////////////////////////

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/shallow_history.hpp>
#include <boost/statechart/deep_history.hpp>

#include <boost/mpl/list.hpp>
#include <boost/shared_ptr.hpp>
#include <smacc/smacc.h>

//#include <boost/test/test_tools.hpp>

#include <stdexcept>
#include <thread>

#include <future>
#include <string>
#include <mutex>

namespace sc = boost::statechart;
using namespace smacc;

struct EvToDeep : sc::event<EvToDeep>
{
};
struct Ev1 : sc::event<Ev1>
{
};
struct Ev2 : sc::event<Ev2>
{
};
struct Ev3 : sc::event<Ev3>
{
};
struct EvFail : sc::event<EvFail>
{
};

struct MsRunMode;
struct St1;
struct St2;
struct St3;

struct MsRecoveryMode;

struct SmDanceBot : smacc::SmaccStateMachineBase<SmDanceBot, MsRunMode>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    void unconsumed_event(const sc::event_base &)
    {
        throw std::runtime_error("Event was not consumed!");
    }
};

//---------------------------------------------------------------------------------------------
struct MsRunMode : SmaccState<MsRunMode, SmDanceBot, St1, sc::has_full_history>
{
    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("MS RUN MODE");
    }
};

struct St1 : SmaccState<St1, MsRunMode>
{
    typedef mpl::list<smacc::transition<Ev1, St2, smacc::SUCCESS>,
                      smacc::transition<EvFail, MsRecoveryMode, smacc::ABORT>>
        reactions;
    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St1");
        std::async(std::launch::async, [=]() {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            this->postEvent(new Ev1());
        });
    }
};

struct St2 : SmaccState<St2, MsRunMode>
{
    static int counter;
    typedef mpl::list<smacc::transition<Ev2, St3, smacc::SUCCESS>,
                      smacc::transition<EvFail, MsRecoveryMode, smacc::ABORT>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St2");
        ROS_INFO_STREAM("counter: " << counter);
        if (counter == 0)
            std::async(std::launch::async, [=]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                this->postEvent(new EvFail());
            });
        else
        std::async(std::launch::async, [=]()
            {
                std::this_thread::sleep_for(std::chrono::seconds(3));
                this->postEvent(new Ev2());
            });

        counter++;
    }
};

struct St3 : SmaccState<St3, MsRunMode>
{
    typedef smacc::transition<EvFail, MsRecoveryMode, smacc::ABORT> reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St3");
    }
};

//---------------------------------------------------------------------------------------------
struct MsRecoveryMode : SmaccState<MsRecoveryMode, SmDanceBot>
{
    typedef mpl::list<

        smacc::transition<EvToDeep, sc::deep_history<St1>, SUCCESS>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("Recovery");
        std::async(std::launch::async, [=]() {
                std::this_thread::sleep_for(std::chrono::seconds(3));
        this->postEvent(new EvToDeep()); });
    }
};

int St2::counter = 0;