/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/smacc_signal_detector.h>
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
* poll()
******************************************************************************************************************
*/
void SignalDetector::pollOnce()
{
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