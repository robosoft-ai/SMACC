#include <gtest/gtest.h>

#include <smacc/smacc.h>

smacc::SignalDetector *signalDetector_;

int loop_rate_hz = 100;
bool end_ = false;

void myPollingLoop()
{
    ros::NodeHandle nh("~");

    if (!nh.param("signal_detector_loop_freq", loop_rate_hz))
    {
    }

    nh.setParam("signal_detector_loop_freq", loop_rate_hz);

    ROS_INFO_STREAM("[SignalDetector] loop rate hz:" << loop_rate_hz);

    int i = 0;
    ros::Rate r(loop_rate_hz);
    while (ros::ok() && !end_ && i++ < 1000) 
    {
        // std::cout << "***** " << (end_ ? "END" : "!END") << " " << i << std::endl;
        ROS_INFO_STREAM_THROTTLE(10, "[SignalDetector] heartbeat");
        signalDetector_->pollOnce();
        ros::spinOnce();
        r.sleep();
    }
}

namespace simple_state_machine {

class StTock;

class SimpleStateMachine : public smacc::SmaccStateMachineBase<SimpleStateMachine, StTock>
{
public:
    SimpleStateMachine(my_context ctx, smacc::SignalDetector *signalDetector) 
        : smacc::SmaccStateMachineBase<SimpleStateMachine, StTock>(ctx, signalDetector)
    {}

    virtual void onInitialize()
    {
        setGlobalSMData("time", 2);
        std::string msg = "Fake State Machine";
        setGlobalSMData("message", msg);
    }

    virtual void unconsumed_event(const sc::event_base &evt)
    {
        ROS_WARN_STREAM("**** UNCONSUMED EVENT ****  " << smacc::demangleSymbol(typeid(evt).name()));
    } 

private:
};

}  // namespace simple_state_machine

#include <ssm/StTock.h>

void timerCallback(const ros::TimerEvent &event)
{
    std::cout << "Timer" << std::endl;
    end_ = true;
}

void my_run()
{
    SmaccFifoScheduler scheduler1(true);
    signalDetector_ = new smacc::SignalDetector(&scheduler1);
    SmaccFifoScheduler::processor_handle sm =
        scheduler1.create_processor<simple_state_machine::SimpleStateMachine>(signalDetector_);
    signalDetector_->setProcessorHandle(sm);
    scheduler1.initiate_processor(sm);
    boost::thread otherThread(boost::bind(&sc::fifo_scheduler<>::operator(), &scheduler1, 0));
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(5.0), timerCallback);

    myPollingLoop();
    scheduler1.terminate();

    std::cout << "waiting closing state machine thread" << std::endl;
    std::cout << "exiting process" << std::endl;
}

TEST(SSM_TESTS, testStateMachine1)
{
    std::cout << "testStateMachine1" << std::endl;

    int argc = 1;
    char *argv[1];
    argv[0] = (char*)"my test";
    ros::init(argc,(char**) argv, "ssm");
    ros::NodeHandle nh;

    my_run();

    ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
