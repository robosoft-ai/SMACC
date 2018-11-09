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
            
        // runs the polling loop into a thread
        void runThread();

        // waits the polling thread to end
        void join();
        
        void pollOnce();

        // prints the current state of the signal detector into a string
        void toString(std::stringstream& ss);

        void pollingLoop();

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

        // loop frequency of the signal detector (to check answers from actionservers)
        double loop_rate_hz;

        friend class ISmaccStateMachine;
};
}
