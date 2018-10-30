#pragma once

#include "smacc/smacc_action_client.h"
#include <queue>

namespace smacc
{
// Smacc Action Clients (AKA resources or plugins) can inherit from this object (that works as a template .h library)
template <typename ActionType>
class SmaccActionClientBase: public ISmaccActionClient
{
    public:

    // inside this macro you can find the typedefs for Goal and other types
    ACTION_DEFINITION(ActionType);
    typedef actionlib::SimpleActionClient<ActionType> ActionClient ;
    typedef actionlib::SimpleActionClient<ActionType> GoalHandle;

    SmaccActionClientBase(std::string action_client_namespace, int feedback_queue_size=10)
        :ISmaccActionClient(action_client_namespace),
        client_(action_client_namespace,false) 
    {
        this->feedback_queue_size_= feedback_queue_size;
    }

    virtual ~SmaccActionClientBase()
    {
    }

    void waitForActionServer()
    {
        //ROS_INFO("waiting for action server: %s", this->getName().c_str());
        //client_.waitForServer();
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


    void onTransition(GoalHandle goalRequest)
    {
    }

    virtual bool hasFeedback() override
    {
        return !feedback_queue_.empty();
    }

    bool popFeedback(Feedback& feedback_msg)
    {
        if(feedback_queue_.empty())
        {
            feedback_msg = feedback_queue_.front();
            feedback_queue_.pop_front();
            return true;
        }
        else
        {
            return false;
        }
    }

    void onFeedback(GoalHandle goalRequest, const FeedbackConstPtr & feedback)
    {
        feedback_queue_.push_back(*feedback);
        if(feedback_queue_.size()>this->feedback_queue_size_)
        {
            feedback_queue_.pop_front();
        }
    }

    void sendGoal(Goal& goal)
    {
        ROS_INFO_STREAM("Sending goal to actionserver located in " << this->name_ <<"\"");
        
        if(!client_.isServerConnected())
        {
            ROS_INFO("%s [at %s]: not connected with actionserver, waiting ..." , this->getName().c_str(), this->getNamespace().c_str());
            client_.waitForServer();
        }

        ROS_INFO_STREAM(this->getName()<< ": Goal Value: " << std::endl << goal);
        client_.sendGoal(goal);

        stateMachine_->registerActionClientRequest(this);
    }

    protected:
        ActionClient client_;
        int feedback_queue_size_;
        std::queue<Feedback> feedback_queue_;
};
}