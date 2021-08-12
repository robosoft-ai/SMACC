/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <boost/optional.hpp>
#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{

class CbEmergencyStop : public CbMoveBaseClientBehaviorBase
{
public:

  CbEmergencyStop();

  virtual void onEntry() override;
  virtual void onExit() override;

private:
};
}  // namespace cl_move_base_z
