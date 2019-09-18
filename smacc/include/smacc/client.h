/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>

namespace smacc
{
class ISmaccClient: public ISmaccComponent
{
public:
    ISmaccClient();
    virtual ~ISmaccClient();
};
}