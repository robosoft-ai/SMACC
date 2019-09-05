/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc_core/component.h>
#include <actionlib/client/simple_action_client.h>

namespace smacc
{
using namespace actionlib;

// This class interface shows the basic set of methods that
// a SMACC "resource" or "plugin" Action Client has
class ISmaccActionClient: public ISmaccComponent
{
public:

    ISmaccActionClient();

    // the destructor. This is called when the object is not 
    // referenced anymore by its owner
    virtual ~ISmaccActionClient();

    virtual void init(ros::NodeHandle& nh) override;

    virtual void init(ros::NodeHandle& nh, std::string) override;
    
    // return the current state of the actionclient
    virtual SimpleClientGoalState getState()=0;

    // checks if there is some feedback message waiting
    virtual bool hasFeedback() = 0;

    // gets the ros path of the action
    inline std::string getNamespace() const
    {
        return name_;
    }

protected:
    virtual void postEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle)=0;

    // used internally by the Signal detector
    virtual void postFeedbackEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle)=0;

    // the ros path where the action is located
    std::string name_;

    friend class SignalDetector;
};
}