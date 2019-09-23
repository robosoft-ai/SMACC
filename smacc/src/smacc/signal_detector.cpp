/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/signal_detector.h>
#include <smacc/interface_components/smacc_action_client_base.h>

namespace smacc
{
/**
******************************************************************************************************************
* SignalDetector()
******************************************************************************************************************
*/
SignalDetector::SignalDetector(SmaccScheduler *scheduler)
{
    scheduler_ = scheduler;
    loop_rate_hz = 10.0;
    end_= false;
}

/**
******************************************************************************************************************
* registerActionClientRequest()
******************************************************************************************************************
*/
void SignalDetector::registerActionClientRequest(ISmaccActionClient *actionClientRequestInfo)
{
    ROS_INFO("Signal detector is aware of the '-- %s -- action client request'", actionClientRequestInfo->getName().c_str());
    openRequests_.push_back(actionClientRequestInfo);
    ROS_INFO("Added to the opened requests list");
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void SignalDetector::initialize(ISmaccStateMachine *stateMachine)
{
    smaccStateMachine_ = stateMachine;
}

/**
******************************************************************************************************************
* setProcessorHandle()
******************************************************************************************************************
*/
void SignalDetector::setProcessorHandle(SmaccScheduler::processor_handle processorHandle)
{
    processorHandle_ = processorHandle;
}

/**
******************************************************************************************************************
* runThread()
******************************************************************************************************************
*/
void SignalDetector::runThread()
{
    signalDetectorThread_ = boost::thread(boost::bind(&SignalDetector::pollingLoop, this));
}

/**
******************************************************************************************************************
* join()
******************************************************************************************************************
*/
void SignalDetector::join()
{
    signalDetectorThread_.join();
}

/**
******************************************************************************************************************
* stop()
******************************************************************************************************************
*/
void SignalDetector::stop()
{
    end_=true;
}

/**
******************************************************************************************************************
* notifyFeedback()
******************************************************************************************************************
*/
void SignalDetector::notifyFeedback(ISmaccActionClient *client)
{
    //ROS_INFO("Notify feedback");
    //boost::intrusive_ptr< EvActionFeedback > actionFeedbackEvent = new EvActionFeedback();
    //actionFeedbackEvent->client = client;

    client->postFeedbackEvent(scheduler_, processorHandle_);

    //ROS_INFO("Sending feedback event");
    //scheduler_->queue_event(processorHandle_, actionFeedbackEvent);
}

/**
******************************************************************************************************************
* finalizeRequest()
******************************************************************************************************************
*/
void SignalDetector::finalizeRequest(ISmaccActionClient *client)
{
    ROS_INFO("SignalDetector: Finalizing actionlib request: %s. RESULT: %s", client->getName().c_str(), client->getState().toString().c_str());
    auto it = find(openRequests_.begin(), openRequests_.end(), client);

    if (it != openRequests_.end())
    {
        openRequests_.erase(it);
    }

    //boost::intrusive_ptr< IActionResult> actionClientResultEvent = client->createActionResultEvent();
    //actionClientResultEvent->client = client;

    ROS_INFO("SignalDetector: action lib result obtained, posting event");
    client->postEvent(scheduler_, processorHandle_);
    // SmaccScheduler
    //SmaccScheduler::processor_handle
    //scheduler_->queue_event(processorHandle_, actionClientResultEvent);
}

/**
******************************************************************************************************************
* toString()
******************************************************************************************************************
*/
void SignalDetector::toString(std::stringstream &ss)
{
    ss << "--------" << std::endl;
    ss << "Open requests" << std::endl;
    for (ISmaccActionClient *smaccActionClient : this->openRequests_)
    {
        auto state = smaccActionClient->getState().toString();
        ss << smaccActionClient->getName() << ": " << state << std::endl;
    }
    ss << "--------";
}

/**
******************************************************************************************************************
* poll()
******************************************************************************************************************
*/
void SignalDetector::pollOnce()
{
    for (ISmaccActionClient *smaccActionClient : openRequests_)
    {
        // check feedback messages
        if (smaccActionClient->hasFeedback())
        {
            notifyFeedback(smaccActionClient);
        }

        // check result
        auto state = smaccActionClient->getState();
        if (state.isDone())
        {
            finalizeRequest(smaccActionClient);
        }
    }
}

/**
******************************************************************************************************************
* pollingLoop()
******************************************************************************************************************
*/
void SignalDetector::pollingLoop()
{
    ros::NodeHandle nh("~");

    if (!nh.param("signal_detector_loop_freq", this->loop_rate_hz))
    {
    }

    nh.setParam("signal_detector_loop_freq", this->loop_rate_hz);

    ros::Rate r(loop_rate_hz);

    ROS_INFO_STREAM("[SignalDetector] loop rate hz:" << loop_rate_hz);
    while (ros::ok() && !end_)
    {
        ROS_INFO_STREAM_THROTTLE(10, "[SignalDetector] heartbeat");

        pollOnce();
        ros::spinOnce();
        r.sleep();
    }
}
} // namespace smacc