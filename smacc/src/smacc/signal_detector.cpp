/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/smacc_signal_detector.h>
#include <smacc/client_bases/smacc_action_client_base.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
/**
******************************************************************************************************************
* SignalDetector()
******************************************************************************************************************
*/
SignalDetector::SignalDetector(SmaccFifoScheduler *scheduler)
{
    scheduler_ = scheduler;
    loop_rate_hz = 10.0;
    end_ = false;
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void SignalDetector::initialize(ISmaccStateMachine *stateMachine)
{
    smaccStateMachine_ = stateMachine;
    lastState_ = nullptr;

    findUpdatableClients();
}

/**
******************************************************************************************************************
* findUpdatableClients()
******************************************************************************************************************
*/
void SignalDetector::findUpdatableClients()
{
    this->updatableClients_.clear();
    for (auto pair : this->smaccStateMachine_->getOrthogonals())
    {
        auto &orthogonal = pair.second;
        auto &clients = orthogonal->getClients();

        for (auto &client : clients)
        {
            auto updatableClient = dynamic_cast<ISmaccUpdatable *>(client.get());

            if (updatableClient != nullptr)
            {
                this->updatableClients_.push_back(updatableClient);
            }
        }
    }
}

/**
******************************************************************************************************************
* findUpdatableSubstateBehaviors()
******************************************************************************************************************
*/
void SignalDetector::findUpdatableBehaviors()
{
    this->updatableSubstateBehaviors_.clear();
    for (auto pair : this->smaccStateMachine_->getOrthogonals())
    {
        auto &orthogonal = pair.second;
        auto *currentBehavior = orthogonal->getCurrentBehavior();

        if (currentBehavior == nullptr)
            continue;

        ISmaccUpdatable *updatableSubstateBehavior = dynamic_cast<ISmaccUpdatable *>(currentBehavior);

        if (updatableSubstateBehavior != nullptr)
        {
            this->updatableSubstateBehaviors_.push_back(updatableSubstateBehavior);
        }
    }
}

/**
******************************************************************************************************************
* setProcessorHandle()
******************************************************************************************************************
*/
void SignalDetector::setProcessorHandle(SmaccFifoScheduler::processor_handle processorHandle)
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
    end_ = true;
}

/**
******************************************************************************************************************
* poll()
******************************************************************************************************************
*/
void SignalDetector::pollOnce()
{    
    for (auto *updatableClient : this->updatableClients_)
    {
        ROS_DEBUG("pollOnce update client call: ");
        updatableClient->update();
    }

    try
    {
        smaccStateMachine_->lockStateMachine();
        auto *currentState = smaccStateMachine_->getCurrentState();
        if(currentState!=nullptr)
        {
                if (currentState != this->lastState_)
                {
                    // we are in a new state, refresh the updatable substate behaviors table
                    this->findUpdatableBehaviors();
                }

                this->lastState_ = currentState;

                for (auto *updatableBehavior : this->updatableSubstateBehaviors_)
                {
                    ROS_DEBUG("pollOnce update substate behavior call: ");
                    updatableBehavior->update();
                }
        }
    }
    catch(...)
    {

    }
    smaccStateMachine_->unlockStateMachine();
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