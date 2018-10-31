#pragma once

#include "smacc/smacc_action_client.h"
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

    SmaccActionClientBase(std::string action_client_namespace, int feedback_queue_size=10)
        :ISmaccActionClient(action_client_namespace),
        client_(action_client_namespace,false) 
    {
        this->feedback_queue_size_= feedback_queue_size;
    }

    virtual ~SmaccActionClientBase()
    {
    }

    virtual std::string getName() const =0;

    virtual void cancelGoal() override
    {
        ROS_INFO("Cancelling goal of %s", this->getName().c_str());
        client_.cancelGoal();
    }

    virtual SimpleClientGoalState getState() override
    {
        return client_.getState();
    }

    virtual bool hasFeedback() override
    {
        return !feedback_queue_.empty();
    }

    Feedback getFeedbackMessage(const EvActionFeedback& ev)
    {
        return boost::any_cast<Feedback>(ev.feedbackMessage);
    }

    void sendGoal(Goal& goal)
    {
        ROS_INFO_STREAM("Sending goal to actionserver located in " << this->name_ <<"\"");
        
        if(!client_.isServerConnected())
        {
            ROS_INFO("%s [at %s]: not connected with actionserver, waiting ..." , getName().c_str(), getNamespace().c_str());
            client_.waitForServer();
        }

        ROS_INFO_STREAM(getName()<< ": Goal Value: " << std::endl << goal);

        SimpleDoneCallback done_cb ;
        SimpleActiveCallback active_cb;
        SimpleFeedbackCallback feedback_cb = boost::bind(&SmaccActionClientBase<ActionType>::onFeedback,this,_1);

        client_.sendGoal(goal,done_cb,active_cb,feedback_cb);

        stateMachine_->registerActionClientRequest(this);
    }

protected:
    ActionClient client_;
    int feedback_queue_size_;
    std::list<Feedback> feedback_queue_;

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

    // here it is assigned one feedback message to one smacc event
    virtual bool popFeedback(EvActionFeedback& ev) override
    {
        if(!feedback_queue_.empty())
        {
            ROS_DEBUG("[%s]Popping FEEDBACK MESSAGE, Queue Size: %ld", this->getName().c_str(), feedback_queue_.size());
            Feedback feedback_msg = feedback_queue_.front();
            feedback_queue_.pop_front();
            ROS_DEBUG("[%s]popped FEEDBACK MESSAGE, Queue Size: %ld", this->getName().c_str(), feedback_queue_.size());
            ev.feedbackMessage = feedback_msg;
        
            return true;
        }
        else
        {
            return false;
        };
    }

    friend class SignalDetector;
};
}