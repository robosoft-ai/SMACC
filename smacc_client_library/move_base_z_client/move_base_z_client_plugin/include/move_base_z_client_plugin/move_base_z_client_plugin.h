/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_action_client_base.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <planner_switcher/planner_switcher.h>

namespace smacc
{
class WaypointNavigator;

class ClMoveBaseZ : public SmaccActionClientBase<move_base_msgs::MoveBaseAction>
{
    typedef SmaccActionClientBase<move_base_msgs::MoveBaseAction> Base;

public:
    typedef SmaccActionClientBase<move_base_msgs::MoveBaseAction>::ResultConstPtr ResultConstPtr;

    std::shared_ptr<planner_switcher::PlannerSwitcher> plannerSwitcher_;

    ClMoveBaseZ();

    virtual ~ClMoveBaseZ();

    virtual void initialize() override;

    virtual std::string getName() const override;    
};

} // namespace smacc