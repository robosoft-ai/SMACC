#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include "smacc/common.h"

namespace smacc
{
class SignalDetector
{ 
    public:
        SignalDetector(SmaccScheduler* scheduler);

        void initialize(ISmaccStateMachine* stateMachine);

        void setProcessorHandle(SmaccScheduler::processor_handle processorHandle);

        void registerActionClientRequest(ISmaccActionClient* actionClientRequestInfo);
            
        void runThread();

        void join();

        void simulateResponses();
        
        void poll();

        void toString(std::stringstream& ss);

        void pollingLoop();

        void finalizeRequest(ISmaccActionClient*);
    
    private:
        SmaccScheduler* scheduler_;
        SmaccScheduler::processor_handle processorHandle_;
        std::vector<ISmaccActionClient*> openRequests_;
        boost::thread signalDetectorThread_ ;
        ISmaccStateMachine* smaccStateMachine_;
};
}
