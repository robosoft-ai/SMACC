#include "states/navigate_to_radial_start.h"
#include "states/rotate_ten_degrees.h"
#include "states/return_to_radial_start.h"
#include "states/navigate_to_end_point.h"
#include <boost/thread.hpp>
//------------------------------------------------------------------------------

char GetKey()
{
  char key;
  std::cin >> key;
  return key;
}

class SignalDetector
{ 
    public:
        SignalDetector(SmaccScheduler* scheduler, SmaccScheduler::processor_handle smaccStateMachine, std::shared_ptr<boost::signals2::signal<void (std::string actionClientRequestInfo) >> requestSignal)
        {
            scheduler_ = scheduler;
            smaccStateMachine_ = smaccStateMachine;
            requestSignal->connect(boost::bind(&SignalDetector::onActionClientRequest, this,_1));
        }

        void onActionClientRequest(std::string actionClientRequestInfo)
        {
            ROS_INFO("Signal detector is aware of the '%s action client request'", actionClientRequestInfo.c_str());
            openedRequests_.push_back(actionClientRequestInfo);
        }

        void runThread()
        {
            signalDetectorThread_ = boost::thread( boost::bind(&SignalDetector::pollingLoop, this ));
        }

        void join()
        {
            signalDetectorThread_.join();
        }

        void simulateResponses()
        {
            //simualte processing
            ROS_INFO("Simulating action server execution ... 5 secs");
            
            ros::Rate r(1);
            for (int i =0; i< 5 && ros::ok();i++)
            {
                ros::spinOnce();
                r.sleep();
                ROS_INFO(".");
            }

            boost::intrusive_ptr< EvActionClientSuccess > actionClientSuccessEvent = new EvActionClientSuccess();
            //actionClientSuccessEvent->target = openedRequests_.front();
            actionClientSuccessEvent->target = "move_base";
            
            ROS_INFO("Simulated action server success response, sending event to State Machine");
            scheduler_->queue_event(smaccStateMachine_, actionClientSuccessEvent);


            for (int i =0; i< 5 && ros::ok();i++)
            {
                ros::spinOnce();
                r.sleep();
                ROS_INFO(".");
            }

            boost::intrusive_ptr< EvActionClientSuccess > actionClientSuccessEvent2 = new EvActionClientSuccess();
            //actionClientSuccessEvent->target = openedRequests_.front();
            actionClientSuccessEvent->target = "reel";
            
            ROS_INFO("Simulated action server success response, sending event to State Machine");
            scheduler_->queue_event(smaccStateMachine_, actionClientSuccessEvent2);
        }

    private:
        boost::thread signalDetectorThread_;
        SmaccScheduler* scheduler_;
        SmaccScheduler::processor_handle smaccStateMachine_;
        std::vector<std::string> openedRequests_;

        void pollingLoop()
        {
            ros::Rate r(1);
            while (ros::ok())
            {
                ros::spinOnce();
                r.sleep();
                ROS_INFO(".");
            }
        }   
};


int main(int argc, char** argv)
{
    ros::init(argc,argv, "radial_test_state_machine");
    ros::NodeHandle nh;

    ROS_INFO("Hello World!");

    auto shared_request_signal = std::make_shared<boost::signals2::signal<void (std::string actionClientRequestInfo) >>();

    SmaccScheduler scheduler1( true );
    SmaccScheduler::processor_handle smaccStateMachine = scheduler1.create_processor< SmaccStateMachine >(shared_request_signal);

    scheduler1.initiate_processor( smaccStateMachine);


    SignalDetector signalDetector(&scheduler1, smaccStateMachine, shared_request_signal);

    //start the signalDetector thread
    //signalDetector.runThread();

    // start the state machine thread
    boost::thread otherThread( boost::bind(
    &sc::fifo_scheduler<>::operator(), &scheduler1, 0 ) );


    signalDetector.simulateResponses();

    //simulating request delay
    //ros::Duration(1).sleep();

    //boost::intrusive_ptr< EvActionClientSuccess > actionClientSuccessEvent2 = new EvActionClientSuccess();
    //scheduler1.queue_event( smaccStateMachine, actionClientSuccessEvent2);

    // waiting end of state machine
    ros::Duration(10).sleep();
    //otherThread.join();
    //ROS_INFO("State machine thread finished");

    //GetKey();
}

void SmaccStateMachine::initiate_impl() 
{
    ROS_INFO("initiate_impl");
    sc::state_machine< SmaccStateMachine, NavigateToRadialStart::State, SmaccAllocator >::initiate();
}

SmaccStateMachine::SmaccStateMachine( my_context ctx,std::shared_ptr<boost::signals2::signal<void (std::string actionClientRequestInfo) >> requestSignal) :
sc::asynchronous_state_machine<SmaccStateMachine, NavigateToRadialStart::State, SmaccScheduler, SmaccAllocator >(ctx)
, onNewActionClientRequest(requestSignal)
{
    ROS_INFO("Creating state machine");
}