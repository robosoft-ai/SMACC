/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/client.h>
#include <actionlib/client/simple_action_client.h>

namespace smacc
{
using namespace actionlib;

// This class interface shows the basic set of methods that
// a SMACC "resource" or "plugin" Action Client has
class ISmaccActionClient: public ISmaccClient
{
public:

    ISmaccActionClient();

    // The destructor. This is called when the object is not 
    // referenced anymore by its owner
    virtual ~ISmaccActionClient();
    
    // Returns the current state of the actionclient...
    virtual SimpleClientGoalState getState()=0;

    // Checks if there is some feedback message waiting...
    virtual bool hasFeedback() = 0;

    // Gets the ros path of the action...
    inline std::string getNamespace() const
    {
        return name_;
    }

protected:
    virtual void postEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle)=0;

    // Used internally by the Signal detector
    virtual void postFeedbackEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle)=0;

    // The ros path where the action is located
    std::string name_;

    friend class SignalDetector;
};
}