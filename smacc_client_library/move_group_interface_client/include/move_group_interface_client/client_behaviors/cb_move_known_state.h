/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/client_behaviors/cb_move_joints.h>
#include <map>
#include <string>

namespace move_group_interface_client
{
class CbMoveKnownState : public CbMoveJoints
{
public:
  CbMoveKnownState(std::string pkg, std::string config_path);

  private:
  static std::map<std::string, double> loadJointStatesFromFile(std::string pkg, std::string filepath);
};
}  // namespace move_group_interface_client
