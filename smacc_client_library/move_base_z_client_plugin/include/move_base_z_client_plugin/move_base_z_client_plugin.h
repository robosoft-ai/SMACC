/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_action_client_base.h>

#include <move_base_msgs/MoveBaseAction.h>

namespace cl_move_base_z
{

class ClMoveBaseZ : public smacc::client_bases::SmaccActionClientBase<move_base_msgs::MoveBaseAction>
{
    typedef SmaccActionClientBase<move_base_msgs::MoveBaseAction> Base;

public:
    typedef SmaccActionClientBase<move_base_msgs::MoveBaseAction>::ResultConstPtr ResultConstPtr;

    ClMoveBaseZ(std::string moveBaseName="/move_base");

    virtual ~ClMoveBaseZ();

    virtual void initialize() override;

    virtual std::string getName() const override;
};

} // namespace smacc