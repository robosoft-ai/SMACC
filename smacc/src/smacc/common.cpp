/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include "smacc/common.h"
#include "smacc/interface_components/smacc_action_client_base.h"

namespace smacc
{
actionlib::SimpleClientGoalState IActionResult::getResultState() const
{
    return client->getState();
}

std::string cleanShortTypeName(const std::type_info& tinfo)
{
    std::string fullclassname = demangleSymbol(tinfo.name());
    //ROS_INFO("State full classname: %s", fullclassname.c_str());

    std::vector<std::string> strs;
    boost::split(strs,fullclassname,boost::is_any_of("::"));
    std::string classname = strs.back();
    //ROS_INFO("State classname: %s", classname.c_str());
    return classname;
}
}
