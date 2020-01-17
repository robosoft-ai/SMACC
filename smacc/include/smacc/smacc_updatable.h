/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

namespace smacc
{
class ISmaccUpdatable
{
public:
    virtual void update() = 0;
};
} // namespace smacc