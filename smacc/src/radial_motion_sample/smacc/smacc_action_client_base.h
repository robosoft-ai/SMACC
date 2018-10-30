#pragma once

#include "smacc/smacc_action_client.h"

namespace smacc
{
// Smacc Action Clients (AKA resources or plugins) can inherit from this object (that works as a template .h library)
template <typename ActionType>
class SmaccActionClientBase: public ISmaccActionClient
{
    public:

    ACTION_DEFINITION(ActionType);
    typedef actionlib::SimpleActionClient<ActionType> ActionClient ;

    SmaccActionClientBase(std::string action_client_namespace)
        :ISmaccActionClient(action_client_namespace),
        client_(action_client_namespace,false) 
    {
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
        ActionClient  client_;
};
}