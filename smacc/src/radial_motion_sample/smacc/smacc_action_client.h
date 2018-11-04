#pragma once

#include <actionlib/client/simple_action_client.h>
#include "smacc/common.h"
#include "smacc/state_machine.h"
#include "smacc/smacc_state_machine.h"

namespace smacc
{
using namespace actionlib;

// This class interface shows the basic set of methods that
// a SMACC "resource" or "plugin" Action Client has
class ISmaccActionClient
{
public:
    // the destructor. This is called when the object is not 
    // referenced anymore by its owner
    virtual ~ISmaccActionClient();

    // assings the owner of this resource to the given state machine parameter object 
    void setStateMachine(ISmaccStateMachine* stateMachine);

    // returns a custom identifier defined by the specific plugin implementation
    virtual std::string getName() const=0;

    // return the current state of the actionclient
    virtual SimpleClientGoalState getState()=0;

    // cancels the latest goal request
    virtual void cancelGoal() = 0;

    // checks if there is some feedback message waiting
    virtual bool hasFeedback() = 0;

    // gets the ros path of the action
    inline std::string getNamespace() const
    {
        return name_;
    }

protected:
    // used internally by the Signal detector
    virtual bool popFeedback(EvActionFeedback& actionFeedbackEvent)=0;

    // the ros path where the action is located
    std::string name_;

    // a reference to the state machine object that owns this resource
    ISmaccStateMachine* stateMachine_;
    
    // simple factory pattern
    ISmaccActionClient(std::string action_client_namespace);

    friend class SignalDetector;
};
}