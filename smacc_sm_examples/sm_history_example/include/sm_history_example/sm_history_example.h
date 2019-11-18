#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/shallow_history.hpp>
#include <boost/statechart/deep_history.hpp>

#include <boost/mpl/list.hpp>
#include <boost/shared_ptr.hpp>

//#include <boost/test/test_tools.hpp>

#include <stdexcept>
#include <thread>

#include <future>
#include <string>
#include <mutex>

#include <smacc/smacc.h>
#include <smacc/orthogonal.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_odom_tracker/odom_tracker.h>

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

template <typename TEv>
void delayedPostEvent(smacc::ISmaccState *sm, int delayseconds)
{
    std::async(std::launch::async, [=]() {
        std::this_thread::sleep_for(std::chrono::seconds(delayseconds));
        sm->postEvent(new TEv());
    });
}

struct MsRunMode;
struct St1;
struct St2;
struct St3;

struct MsRecoveryMode;

class NavigationOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *client = this->createClient<smacc::SmaccMoveBaseActionClient>();
        client->name_ = "move_base";
        client->initialize();
    }
};

struct SmHistoryExample : smacc::SmaccStateMachineBase<SmHistoryExample, MsRunMode>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<NavigationOrthogonal>();
        int initialcounterValue = 0;
        //this->setGlobalSMData("St2Attempts", initialcounterValue);
    }

    void unconsumed_event(const sc::event_base &)
    {
        throw std::runtime_error("Event was not consumed!");
    }
};

//---------------------------------------------------------------------------------------------
struct MsRunMode : SmaccState<MsRunMode, SmHistoryExample, St1, sc::has_full_history>
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
        delayedPostEvent<Ev1>(this, 7);
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
        if (counter % 2 == 0)
            delayedPostEvent<EvFail>(this, 3);
        else
            delayedPostEvent<Ev2>(this, 3);

        counter++;
        this->setGlobalSMData("St2Attempts", counter);
    }
};

struct St3 : SmaccState<St3, MsRunMode>
{
    typedef mpl::list<smacc::transition<Ev3, St1, smacc::SUCCESS>,
                      smacc::transition<EvFail, MsRecoveryMode, smacc::ABORT>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St3");
        delayedPostEvent<Ev3>(this, 3);
    }
};

//---------------------------------------------------------------------------------------------
struct MsRecoveryMode : SmaccState<MsRecoveryMode, SmHistoryExample>
{
    typedef mpl::list<

        smacc::transition<EvToDeep, sc::deep_history<St1>, SUCCESS>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("Recovery");
        delayedPostEvent<EvToDeep>(this, 3);
    }
};

int St2::counter = 0;