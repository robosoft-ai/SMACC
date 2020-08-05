/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include "cb_move_joints.h"

namespace cl_move_group_interface
{
    class CbMoveLastTrajectoryInitialState : public CbMoveJoints
    {
    public:
        CbMoveLastTrajectoryInitialState();

        virtual ~CbMoveLastTrajectoryInitialState();

        virtual void onEntry() override;
    };
} // namespace cl_move_group_interface