#pragma once

#include <actionlib/client/simple_action_client.h>
#include "smacc/common.h"
#include "smacc/state_machine.h"

namespace smacc
{
    using namespace actionlib;

    // This class interface shows the basic set of methods that
    // a SMACC "resource" or "plugin" Action Client has
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

        inline std::string getNamespace() const
        {
            return name_;
        }
        
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