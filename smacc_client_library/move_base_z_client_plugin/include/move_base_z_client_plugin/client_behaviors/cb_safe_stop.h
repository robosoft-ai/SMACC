/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include "cb_move_base_client_behavior_base.h"

namespace cl_move_base_z
{
class CbSafeStop : public CbMoveBaseClientBehaviorBase
{
public:

    CbSafeStop();

    CbSafeStop(float rotate_degree);

    virtual void onEntry() override;
};
} // namespace cl_move_base_z
