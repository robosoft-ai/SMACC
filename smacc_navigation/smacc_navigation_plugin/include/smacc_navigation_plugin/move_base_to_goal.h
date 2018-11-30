/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_action_client_base.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace smacc
{
class SmaccMoveBaseActionClient: public SmaccActionClientBase<move_base_msgs::MoveBaseAction>
{
    typedef SmaccActionClientBase<move_base_msgs::MoveBaseAction> Base;

    public:
        SmaccMoveBaseActionClient();
        SmaccMoveBaseActionClient(std::string action_server_namespace);
        virtual std::string getName() const override;
        virtual ~SmaccMoveBaseActionClient();
};
}