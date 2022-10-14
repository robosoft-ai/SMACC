
#include <thread>
#include <future>

#include <smacc/smacc.h>

// CLIENT BEHAVIORS
// - no client behavior in this example -

// ORTHOGONALS
#include <sm_viewer_sim/orthogonals/or_navigation.h>

using namespace smacc;

namespace sm_viewer_sim
{
struct MsRecoveryMode;
struct MsRunMode;
struct St1;
struct St2;
struct St3;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
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

template <typename TEv>
void delayedPostEvent(smacc::ISmaccState *sm, int delayseconds)
{
    std::async(std::launch::async, [=]() {
        std::this_thread::sleep_for(std::chrono::seconds(delayseconds));
        sm->postEvent(new TEv());
    });
}

struct SmViewerSim : smacc::SmaccStateMachineBase<SmViewerSim, MsRunMode>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrNavigation>();
        int initialcounterValue = 0;
        //this->setGlobalSMData("St2Attempts", initialcounterValue);
    }

    void unconsumed_event(const sc::event_base &)
    {
        throw std::runtime_error("Event was not consumed!");
    }
};
} // namespace sm_viewer_sim

// MODE STATES
#include <sm_viewer_sim/modestates/ms_run_mode.h>
#include <sm_viewer_sim/modestates/ms_recovery_mode.h>


// STATES
#include <sm_viewer_sim/states/st_st1.h>
#include <sm_viewer_sim/states/st_st2.h>
#include <sm_viewer_sim/states/st_st3.h>
