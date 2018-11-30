
#pragma once

#include <smacc/common.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
class ISmaccComponent
{
public:
    ISmaccComponent();

    virtual ~ISmaccComponent();

    // it is called just after the component is created by the State Machine
    virtual void init(ros::NodeHandle& nh);

    // assings the owner of this resource to the given state machine parameter object 
    void setStateMachine(ISmaccStateMachine* stateMachine);

    // returns a custom identifier defined by the specific plugin implementation
    virtual std::string getName() const;

protected:
    // a reference to the state machine object that owns this resource
    ISmaccStateMachine* stateMachine_;
    
};
}