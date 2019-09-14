#pragma once

#include <smacc/component.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
class SmaccStateBehavior;
class ISmaccState;

//#define SMACC_STATE_BEHAVIOR(BEHAVIOR_CLASS) \
//    SmaccStateBehavior* definesBehavioralSmaccState() \
//    {                                                 \
//      BEHAVIOR_CLASS* behavior;                         \
//      this->requiresComponent(behavior);              \
//      return behavior;                                \
//    }

#define SMACC_STATE_BEHAVIOR                                                     \
    SmaccStateBehavior *definesBehavioralSmaccState()                            \
    {                                                                            \
        std::string shortname = this->getFullName();                             \
        ROS_INFO("trying to get the substate behavior: %s", shortname.c_str());  \
        SmaccStateBehavior *behavior;                                            \
                                                                                 \
        bool found = this->getGlobalSMData(shortname, behavior);                 \
        ROS_INFO("substate behavior '%s' exists? %d", shortname.c_str(), found); \
        return behavior;                                                         \
    }

class SmaccStateBehavior : public smacc::ISmaccComponent
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

    virtual void onEntry()
    {
        ROS_INFO("SmaccStateBehavior %s onEntry", this->getName().c_str());
    }

    virtual bool onExit()
    {
        ROS_INFO("SmaccStateBehavior %s onExit", this->getName().c_str());
        return true;
    }
};
} // namespace smacc