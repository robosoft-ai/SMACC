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
    virtual ~ISmaccActionClient();

    void setStateMachine(ISmaccStateMachine* stateMachine);

    virtual SimpleClientGoalState getState()=0;

    virtual std::string getName() const=0;

    virtual void cancelGoal() = 0;

    inline std::string getNamespace() const
    {
        return name_;
    }
    
protected:
    // the ros path where the action is located
    std::string name_;

    // a reference to the state machine object that owns this resource
    ISmaccStateMachine* stateMachine_;
    
    // simple factory pattern
    ISmaccActionClient(std::string action_client_namespace);
};
}