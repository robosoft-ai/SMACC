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
class ISmaccSubscriber: public ISmaccComponent
{
public:
    ISmaccSubscriber();

    // the destructor. This is called when the object is not 
    // referenced anymore by its owner
    virtual ~ISmaccSubscriber();

    // gets the ros path of the action
    inline std::string getNamespace() const
    {
        return name_;
    }

protected:
    virtual void postEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle)=0;

    // the ros path where the action is located
    std::string name_;

    friend class SignalDetector;

    ros::Subscriber subscriber_;
};
}