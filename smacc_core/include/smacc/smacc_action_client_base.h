/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_action_client.h>
#include <queue>

namespace smacc
{
// Smacc Action Clients (AKA resources or plugins) can inherit from this object 
// inhteriting from this class works as a template .h library. That is why the code
// implementation is located here.
template <typename ActionType>
class SmaccActionClientBase: public ISmaccActionClient
{
    public:

    // inside this macro you can find the typedefs for Goal and other types
    ACTION_DEFINITION(ActionType);
    typedef actionlib::SimpleActionClient<ActionType> ActionClient ;
    typedef actionlib::SimpleActionClient<ActionType> GoalHandle;

    typedef typename ActionClient::SimpleDoneCallback SimpleDoneCallback ;
    typedef typename ActionClient::SimpleActiveCallback SimpleActiveCallback;
    typedef typename ActionClient::SimpleFeedbackCallback SimpleFeedbackCallback;

    SmaccActionClientBase(int feedback_queue_size=10)
        :ISmaccActionClient()
    {
        feedback_queue_size_= feedback_queue_size;
    }

    virtual void init(ros::NodeHandle& nh) override
    {
        init(nh, "");
    }

    virtual void init(ros::NodeHandle& nh, std::string value) override
    {
        ISmaccActionClient::init(nh, value);
        client_ = std::make_shared<ActionClient>(name_,false) ;
    }

    virtual ~SmaccActionClientBase()
    {
    }

    virtual void cancelGoal()
    {
        ROS_INFO("Cancelling goal of %s", this->getName().c_str());
        client_->cancelGoal();
    }

    virtual SimpleClientGoalState getState() override
    {
        return client_->getState();
    }

    virtual bool hasFeedback() override
    {
        return !feedback_queue_.empty();
    }

    void sendGoal(Goal& goal)
    {
        ROS_INFO_STREAM("Sending goal to actionserver located in " << this->name_ <<"\"");
        
        if(!client_->isServerConnected())
        {
            ROS_INFO("%s [at %s]: not connected with actionserver, waiting ..." , getName().c_str(), getNamespace().c_str());
            client_->waitForServer();
        }

        ROS_INFO_STREAM(getName()<< ": Goal Value: " << std::endl << goal);

        SimpleDoneCallback done_cb ;
        SimpleActiveCallback active_cb;
        SimpleFeedbackCallback feedback_cb = boost::bind(&SmaccActionClientBase<ActionType>::onFeedback,this,_1);

        client_->sendGoal(goal,done_cb,active_cb,feedback_cb);

        stateMachine_->registerActionClientRequest(this);
    }

protected:
    std::shared_ptr<ActionClient> client_;

    int feedback_queue_size_;
    std::list<Feedback> feedback_queue_;

    int result_queue_size_;
    std::list<Result> result_queue_;

    void onFeedback(const FeedbackConstPtr & feedback)
    {
        Feedback copy = *feedback;
        feedback_queue_.push_back(copy);
        ROS_DEBUG("FEEDBACK MESSAGE RECEIVED, Queue Size: %ld", feedback_queue_.size());
        if(feedback_queue_.size()> feedback_queue_size_)
        {
            feedback_queue_.pop_front();
        }
    }

    void onResult(const  ResultConstPtr& result)
    {
        Result copy = *result;
        result_queue_.push_back(copy);
        ROS_DEBUG("RESULT MESSAGE RECEIVED, Queue Size: %ld", result_queue_.size());
        if(result_queue_.size()> result_queue_size_)
        {
            result_queue_.pop_front();
        }
    }

    virtual void postEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle) override
    {
        Result result_msg;
        boost::intrusive_ptr< EvActionResult<Result>> actionResultEvent = new EvActionResult<Result>();;
        actionResultEvent->client = this;

        if(!result_queue_.empty())
        {
            ROS_DEBUG("[%s]Popping RESULT MESSAGE, Queue Size: %ld", this->getName().c_str(), result_queue_.size());
            result_msg = result_queue_.front();
            result_queue_.pop_front();
            ROS_DEBUG("[%s]popped RESULT MESSAGE, Queue Size: %ld", this->getName().c_str(), result_queue_.size());
            actionResultEvent->resultMessage = result_msg;
            
            scheduler->queue_event(processorHandle, actionResultEvent);

            auto resultType = actionResultEvent->getResult();
            {
                if(resultType==actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    boost::intrusive_ptr< EvActionSucceded<Result>> successEvent = new EvActionSucceded<Result>();;
                    successEvent->client = this;
                    successEvent->resultMessage = result_msg;
                    scheduler->queue_event(processorHandle, successEvent);
                }
                else if(resultType==actionlib::SimpleClientGoalState::ABORTED)
                {
                    boost::intrusive_ptr< EvActionAborted<Result>> abortedEvent = new EvActionAborted<Result>();;
                    abortedEvent->client = this;
                    abortedEvent->resultMessage = result_msg;
                    scheduler->queue_event(processorHandle, abortedEvent);
                }
                else if(resultType==actionlib::SimpleClientGoalState::REJECTED)
                {
                    boost::intrusive_ptr< EvActionRejected<Result>> rejectedEvent = new EvActionRejected<Result>();;
                    rejectedEvent->client = this;
                    rejectedEvent->resultMessage = result_msg;
                    scheduler->queue_event(processorHandle, rejectedEvent);
                }
                else if(resultType==actionlib::SimpleClientGoalState::PREEMPTED)
                {
                    boost::intrusive_ptr< EvActionPreempted<Result>> preemtedEvent = new EvActionPreempted<Result>();;
                    preemtedEvent->client = this;
                    preemtedEvent->resultMessage = result_msg;
                    scheduler->queue_event(processorHandle, preemtedEvent);
                }
            }
        }
    }
    
    virtual void postFeedbackEvent(SmaccScheduler* scheduler, SmaccScheduler::processor_handle processorHandle) override
    {
        boost::intrusive_ptr< EvActionFeedback<Feedback> > actionFeedbackEvent = new EvActionFeedback<Feedback>();
        actionFeedbackEvent->client = this;

        bool ok = false;
        if(!feedback_queue_.empty())
        {
            ROS_DEBUG("[%s]Popping FEEDBACK MESSAGE, Queue Size: %ld", this->getName().c_str(), feedback_queue_.size());
            Feedback feedback_msg = feedback_queue_.front();
            feedback_queue_.pop_front();
            ROS_DEBUG("[%s]popped FEEDBACK MESSAGE, Queue Size: %ld", this->getName().c_str(), feedback_queue_.size());
            actionFeedbackEvent->feedbackMessage = feedback_msg;
            ok = true;
        }
        else
        {
            ok = false;
        };

        if(ok)
        {
            //ROS_INFO("Sending feedback event");
            scheduler->queue_event(processorHandle, actionFeedbackEvent);
        }
    }    

    friend class SignalDetector;
};
}