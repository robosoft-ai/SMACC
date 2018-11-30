/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include "smacc/common.h"
#include "smacc/smacc_action_client_base.h"

namespace smacc
{
    actionlib::SimpleClientGoalState IActionResult::getResult() const
    {
        return client->getState();
    }
}