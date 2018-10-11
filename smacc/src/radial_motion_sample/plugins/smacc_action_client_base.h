#pragma once

#include "plugins/smacc_action_client.h"

namespace smacc
{
    template <typename ActionType>
    class SmaccActionClientBase: public ISmaccActionClient
    {
        public:

        ACTION_DEFINITION(ActionType);
        typedef actionlib::SimpleActionClient<ActionType> ActionClient ;

        SmaccActionClientBase(std::string action_client_namespace)
            :ISmaccActionClient(action_client_namespace),
            client_(action_client_namespace,true) 
        {
            ROS_INFO("SmaccActionClient base created");
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
            ROS_INFO("Is actionclient connected: %d", client_.isServerConnected());
            client_.waitForServer();
            ROS_INFO("Is actionclient connected: %d", client_.isServerConnected());
            
            ROS_INFO_STREAM("Goal Value: " << std::endl << goal);
            client_.sendGoal(goal);

            ROS_INFO("spinning");
            ros::spin();
            this->stateMachine_->registerActionClientRequest(this);
        }

        protected:
            ActionClient  client_;
    };
}