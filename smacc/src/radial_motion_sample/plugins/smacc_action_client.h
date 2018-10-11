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
            ROS_INFO("ISmaccActionClient created");
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

        virtual void cancelGoal() = 0;

        protected:
            std::string name_;
            ISmaccStateMachine* stateMachine_;

            ISmaccActionClient(std::string action_client_namespace) 
            {
                name_ = action_client_namespace;
                ROS_DEBUG("Creating Action Client %s", action_client_namespace.c_str());
            }            
    };
}