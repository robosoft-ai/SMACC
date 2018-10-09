#include "smacc/signal_detector.h"
#include "plugins/smacc_action_client_base.h"

namespace smacc
{
/**
******************************************************************************************************************
* SignalDetector()
******************************************************************************************************************
*/
    SignalDetector::SignalDetector(SmaccScheduler* scheduler)
    {
        scheduler_ = scheduler;
    }

/**
******************************************************************************************************************
* registerActionClientRequest()
******************************************************************************************************************
*/
    void SignalDetector::registerActionClientRequest(ISmaccActionClient* actionClientRequestInfo)
    {
        ROS_INFO("Signal detector is aware of the '%s action client request'", typeid(actionClientRequestInfo).name());
        openRequests_.push_back(actionClientRequestInfo);
        ROS_INFO("Added to the opened requests list");
    }

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
    void SignalDetector::initialize(ISmaccStateMachine* stateMachine)
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
        signalDetectorThread_ = boost::thread( boost::bind(&SignalDetector::pollingLoop, this ));
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
* simulateResponses()
******************************************************************************************************************
*/
    void SignalDetector::simulateResponses()
    {
        //simualte processing
        ROS_INFO("Simulating action server execution ... 5 secs");
        
        ros::Rate r(1);
        for (int i =0; i< 5 && ros::ok();i++)
        {
            ros::spinOnce();
            r.sleep();
            ROS_INFO(".");
            this->poll();
        }

        ROS_INFO("Simulated action server success response, sending event to State Machine");
        this->finalizeRequest(openRequests_.back());

        for (int i =0; i< 5 && ros::ok();i++)
        {
            ros::spinOnce();
            r.sleep();
            ROS_INFO(".");
            this->poll();
        }

        ROS_INFO("Simulated action server success response, sending event to State Machine");
        this->finalizeRequest(openRequests_.front());
        
    }

/**
******************************************************************************************************************
* finalizeRequest()
******************************************************************************************************************
*/
    void SignalDetector::finalizeRequest(ISmaccActionClient* client)
    {
        ROS_INFO("Finalizing actionlib request: %s", client->getName().c_str());
        auto it = find(openRequests_.begin(),openRequests_.end(),client);

        if (it != openRequests_.end())
        {
            openRequests_.erase(it);
        }

        boost::intrusive_ptr< EvActionClientSuccess > actionClientSuccessEvent = new EvActionClientSuccess();
        actionClientSuccessEvent->client = client;

        scheduler_->queue_event(processorHandle_, actionClientSuccessEvent);
    }

/**
******************************************************************************************************************
* poll()
******************************************************************************************************************
*/
    void SignalDetector::poll()
    {
        std::stringstream ss;
        
        ss << "--------" << std::endl;
        ss << "Open requests" << std::endl;
        for(ISmaccActionClient* smaccActionClient: this->openRequests_)
        {
            auto state = smaccActionClient->getState().toString();
            ss << smaccActionClient->getName() << ": " << state << std::endl;
        }
        ss << "--------";

        ROS_INFO_STREAM(ss.str());            
    }

/**
******************************************************************************************************************
* pollingLoop()
******************************************************************************************************************
*/
    void SignalDetector::pollingLoop()
    {
        ros::Rate r(1);
        while (ros::ok())
        {
            this->poll();
            ros::spinOnce();
            r.sleep();

            for(ISmaccActionClient* smaccActionClient: this->openRequests_)
            {
                auto state = smaccActionClient->getState();
                if(state.isDone())
                {
                    this->finalizeRequest(smaccActionClient);
                }
            }            
        }
    }   
}