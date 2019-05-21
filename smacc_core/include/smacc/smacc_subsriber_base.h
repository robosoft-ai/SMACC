/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_subscriber.h>
#include <queue>

namespace smacc
{
template <typename MessageType>
class SmaccSubscriber: public ISmaccSubscriber
{
    public:

    int messageQueueSize_;
    std::list<MessageType> messageQueue;

    SmaccSubscriber(): messageQueueSize_(1)
    {

    }

    virtual void init(ros::NodeHandle& nh, std::string topic_name)
    {
        ISmaccComponent::init(nh);
        this->topic_name_ = topic_name;
        subscriber_ = nh.subscribe(topic_name, messageQueueSize_ , boost::bind(&SmaccSubscriber::onMessageCallback,this,_1));

    }

    void onMessageCallback(const MessageType& msg)
    {
        MessageType copy = *msg;
        messageQueue.push_back(copy);
        ROS_DEBUG("TOPIC MESSAGE RECEIVED, Queue Size: %ld", messageQueue.size());
        if(messageQueue.size()> messageQueueSize_)
        {
            messageQueueSize_.pop_front();
        }   
    }

    virtual void postEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle) override
    {
        MessageType msg;
        boost::intrusive_ptr< EvTopicMessage<MessageType>> msgEv = new EvTopicMessage<MessageType>();;
        actionResultEvent->client = this;

        if(!result_queue_.empty())
        {
            ROS_DEBUG("[%s]Popping TOPIC MESSAGE, Queue Size: %ld", this->getName().c_str(), messageQueue.size());
            msg = messageQueue.front();
            msg.pop_front();
            ROS_DEBUG("[%s]popped TOPIC MESSAGE, Queue Size: %ld", this->getName().c_str(), messageQueue.size());
            msgEv->message = msg;
            
            scheduler->queue_event(processorHandle, msgEv);
        }
    }
};