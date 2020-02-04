/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/smacc_signal_detector.h>
#include <smacc/client_bases/smacc_action_client_base.h>
#include <smacc/smacc_state_machine.h>
#include <thread>

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
    lastState_ = std::numeric_limits<unsigned long>::quiet_NaN();
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
                ROS_DEBUG_STREAM("Adding updatable client: " << demangleType(typeid(updatableClient)));
                this->updatableClients_.push_back(updatableClient);
            }
        }
    }
}

/**
******************************************************************************************************************
* findUpdatableClientBehaviors()
******************************************************************************************************************
*/
void SignalDetector::findUpdatableBehaviors()
{
    this->updatableClientBehaviors_.clear();
    for (auto pair : this->smaccStateMachine_->getOrthogonals())
    {
        auto &orthogonal = pair.second;
        auto &behaviors = orthogonal->getClientBehaviors();

        for (auto &currentBehavior : behaviors)
        {
            ISmaccUpdatable *updatableClientBehavior = dynamic_cast<ISmaccUpdatable *>(currentBehavior.get());

            if (updatableClientBehavior != nullptr)
            {
                ROS_DEBUG_STREAM("Adding updatable behavior: " << demangleType(typeid(updatableClientBehavior)));
                this->updatableClientBehaviors_.push_back(updatableClientBehavior);
            }
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
    if (smaccStateMachine_ == nullptr)
    {
        ROS_DEBUG("[PollOnce] update but state machine is not yet set.");
        return;
    }

    try
    {
        smaccStateMachine_->lockStateMachine("update behaviors");
        this->findUpdatableClients();
        ROS_DEBUG_STREAM("updatable clients: " << this->updatableClients_.size());

        if (this->updatableClients_.size())
        {
            for (auto *updatableClient : this->updatableClients_)
            {
                ROS_DEBUG_STREAM("[PollOnce] update client call:  " << demangleType(typeid(updatableClient)));
                updatableClient->update();
            }
        }

        long currentStateIndex = smaccStateMachine_->getCurrentStateCounter();

        ROS_DEBUG_STREAM("[PollOnce] update behaviors. checking current state");

        if (smaccStateMachine_->getCurrentState() != nullptr)
        {
            ROS_DEBUG_STREAM("[PollOnce] current state: " << currentStateIndex);
            ROS_DEBUG_STREAM("[PollOnce] last state: " << this->lastState_);

            if (currentStateIndex != 0)
            {
                if (currentStateIndex != this->lastState_)
                {
                    ROS_DEBUG_STREAM("[PollOnce] detected new state, refreshing updatable client behavior table");
                    // we are in a new state, refresh the updatable client behaviors table
                    this->lastState_ = currentStateIndex;
                    this->findUpdatableBehaviors();
                }

                ROS_DEBUG_STREAM("updatable client_behaviors: " << this->updatableClientBehaviors_.size());
                for (auto *updatableBehavior : this->updatableClientBehaviors_)
                {
                    ROS_DEBUG_STREAM("pollOnce update client behavior call: " << demangleType(typeid(*updatableBehavior)));
                    updatableBehavior->update();
                }
            }
        }
    }
    catch (...)
    {
        ROS_ERROR("Exception during Signal Detector update loop.");
    }
    smaccStateMachine_->unlockStateMachine("update behaviors");
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

    ROS_INFO_STREAM("[SignalDetector] loop rate hz:" << loop_rate_hz);

    ros::Rate r(loop_rate_hz);
    while (ros::ok() && !end_)
    {
        ROS_INFO_STREAM_THROTTLE(10, "[SignalDetector] heartbeat");
        pollOnce();
        ros::spinOnce();
        r.sleep();
    }
}
} // namespace smacc