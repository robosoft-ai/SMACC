/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <boost/thread.hpp>
#include <smacc/common.h>

namespace smacc
{
class SignalDetector
{ 
    public:
        SignalDetector(SmaccScheduler* scheduler);

        void initialize(ISmaccStateMachine* stateMachine);

        void setProcessorHandle(SmaccScheduler::processor_handle processorHandle);
            
        // Runs the polling loop into a thread...
        void runThread();

        // Waits for the polling thread to end...
        void join();
        
        // Prints the current state of the signal detector into a string...
        void toString(std::stringstream& ss);

        void pollingLoop();

        void pollOnce();

        template <typename EventType>
        void postEvent(EventType* ev)
        {
            boost::intrusive_ptr< EventType> weakPtrEvent = ev;
            this->scheduler_->queue_event(processorHandle_, weakPtrEvent);
        }

    private:

        void finalizeRequest(ISmaccActionClient* resource);
    
        void notifyFeedback(ISmaccActionClient* resource);

        void registerActionClientRequest(ISmaccActionClient* actionClientRequestInfo);

        SmaccScheduler* scheduler_;

        SmaccScheduler::processor_handle processorHandle_;

        boost::thread signalDetectorThread_ ;
        
        ISmaccStateMachine* smaccStateMachine_;

        // TODO: this should be thread safe since it may be updated from others threads
        std::vector<ISmaccActionClient*> openRequests_;

        // Loop frequency of the signal detector (to check answers from actionservers)
        double loop_rate_hz;

        friend class ISmaccStateMachine;
};
}
