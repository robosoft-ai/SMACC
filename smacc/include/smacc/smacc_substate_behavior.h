#pragma once

#include <smacc/component.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
class SmaccSubStateBehavior;
class ISmaccState;

//#define SMACC_STATE_BEHAVIOR(BEHAVIOR_CLASS) \
//    SmaccSubStateBehavior* definesBehavioralSmaccState() \
//    {                                                 \
//      BEHAVIOR_CLASS* behavior;                         \
//      this->requiresComponent(behavior);              \
//      return behavior;                                \
//    }

#define SMACC_STATE_BEHAVIOR                                                     \
    SmaccSubStateBehavior *definesBehavioralSmaccState()                         \
    {                                                                            \
        std::string shortname = this->getFullName();                             \
        ROS_INFO("trying to get the substate behavior: %s", shortname.c_str());  \
        SmaccSubStateBehavior *behavior;                                         \
                                                                                 \
        bool found = this->getGlobalSMData(shortname, behavior);                 \
        ROS_INFO("substate behavior '%s' exists? %d", shortname.c_str(), found); \
        return behavior;                                                         \
    }

class SmaccSubStateBehavior
{
public:
    // hapens when
    // return true to destroy the object and false to keep it alive

    // a reference to the owner state machine
    ISmaccStateMachine *stateMachine;

    // a reference to the state where the substate behavior is being executed
    ISmaccState *currentState;

    smacc::Orthogonal *currentOrthogonal;

    SmaccSubStateBehavior();

    virtual ~SmaccSubStateBehavior();

    template <typename EventType>
    void postEvent(const EventType &ev)
    {
        if (stateMachine == nullptr)
        {
            ROS_ERROR("The substate behavior cannot post events before being assigned to an orthogonal. Ignoring post event call.");
        }
        else
        {
            stateMachine->postEvent(ev);
        }
    }

    template <typename SmaccClientType>
    void requiresClient(SmaccClientType *&storage, bool verbose = false)
    {
        currentOrthogonal->requiresClient(storage, verbose);
    }

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, bool verbose = false)
    {
        if (stateMachine == nullptr)
        {
            ROS_ERROR("Cannot use the requiresComponent funcionality before asigning the substate behavior to an orthogonal. Try using the OnEntry method to capture required components.");
        }
        else
        {
            stateMachine->requiresComponent(storage, verbose);
        }
    }

    std::string getName() const;

    virtual void onEntry();

    virtual void onExit();
};
} // namespace smacc