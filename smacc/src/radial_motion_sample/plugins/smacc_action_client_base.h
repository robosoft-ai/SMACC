#pragma once

#include <actionlib/client/simple_action_client.h>
#include "smacc/common.h"
#include "smacc/state_machine.h"

namespace smacc
{
    using namespace actionlib;


    class ISmaccActionClient
    {
        public:
        
        ISmaccActionClient()
        {

        }
        
        virtual ~ISmaccActionClient()
        {
        }

        void setStateMachine(ISmaccStateMachine* stateMachine)
        {
            stateMachine_ = stateMachine;
        }

        virtual SimpleClientGoalState getState()=0;

        virtual std::string getName() const=0;

        protected:
            std::string name_;
            ISmaccStateMachine* stateMachine_;

            ISmaccActionClient(std::string action_client_namespace) 
            {
                name_ = action_client_namespace;
                ROS_DEBUG("Creating Action Client %s", action_client_namespace.c_str());
            }            
    };

    //-----------------------------------------------------------------------------------------
    template <typename ActionType>
    class SmaccActionClientBase: public ISmaccActionClient
    {
        public:

        ACTION_DEFINITION(ActionType);
        typedef actionlib::SimpleActionClient<ActionType> Client ;

        SmaccActionClientBase(std::string action_client_namespace)
            :ISmaccActionClient(action_client_namespace),
            client_(action_client_namespace,true) 
        {
        }

        virtual ~SmaccActionClientBase()
        {
        }

        virtual std::string getName() const
        {
            return "";
        }

        void initialize()
        {
            client_.waitForServer();
            ROS_INFO("Initializing action client %s", name_.c_str());
        }

        virtual SimpleClientGoalState getState() override
        {
            return client_.getState();
        }

        void sendGoal(Goal& goal)
        {
            ROS_INFO("Sending goal to actionclient");
            client_.sendGoal(goal);
            this->stateMachine_->registerActionClientRequest(this);
        }

        protected:
            Client  client_;
    };
}