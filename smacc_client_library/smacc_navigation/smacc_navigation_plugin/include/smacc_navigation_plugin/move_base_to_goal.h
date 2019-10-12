/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/interface_components/smacc_action_client_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <smacc_planner_switcher/planner_switcher.h>

namespace smacc
{
class SmaccMoveBaseActionClient : public SmaccActionClientBase<SmaccMoveBaseActionClient, move_base_msgs::MoveBaseAction>
{
    typedef SmaccActionClientBase<SmaccMoveBaseActionClient, move_base_msgs::MoveBaseAction> Base;

public:
    std::shared_ptr<smacc_planner_switcher::PlannerSwitcher> plannerSwitcher_;

    SmaccMoveBaseActionClient();
    virtual void initialize() override;

    virtual std::string getName() const override;
    virtual ~SmaccMoveBaseActionClient();
};
} // namespace smacc