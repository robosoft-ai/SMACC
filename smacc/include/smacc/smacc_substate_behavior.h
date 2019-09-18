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
    SmaccSubStateBehavior *definesBehavioralSmaccState()                            \
    {                                                                            \
        std::string shortname = this->getFullName();                             \
        ROS_INFO("trying to get the substate behavior: %s", shortname.c_str());  \
        SmaccSubStateBehavior *behavior;                                            \
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

    ISmaccStateMachine *stateMachine;
    ISmaccState *currentState;

    template <typename EventType>
    void postEvent(const EventType &ev)
    {
        stateMachine->postEvent(ev);
    }

    template <typename SmaccComponentType>
    void requiresComponent(SmaccComponentType *&storage, ros::NodeHandle nh = ros::NodeHandle(), std::string value = "")
    {
        stateMachine->requiresComponent(storage, nh, value);
    }

    std::string getName() const
    {
        return demangleSymbol(typeid(*this).name());
    }

    virtual void onEntry()
    {
        ROS_INFO("SmaccSubStateBehavior %s onEntry", this->getName().c_str());
    }

    virtual bool onExit()
    {
        ROS_INFO("SmaccSubStateBehavior %s onExit", this->getName().c_str());
        return true;
    }
};
} // namespace smacc