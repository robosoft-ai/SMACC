/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>
#include <map>
#include <string>

namespace moveit_z_client
{
class CbMoveKnownState : public smacc::SmaccClientBehavior
{
protected:
    ClMoveGroup *movegroupClient_;

public:
    std::string statename;

    CbMoveKnownState();
    CbMoveKnownState(std::string statename);
    virtual void onEntry() override;
    virtual void onExit() override;
};
} // namespace moveit_z_client
