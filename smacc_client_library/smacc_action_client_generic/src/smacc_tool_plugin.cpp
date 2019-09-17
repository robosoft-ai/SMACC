/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc_action_client_generic/smacc_tool_plugin.h>
//#include <pluginlib/class_list_macros.h>


namespace smacc
{

SmaccToolActionClient::SmaccToolActionClient()
{

}

std::string SmaccToolActionClient::getName() const
{
    return "TOOL ACTION CLIENT";
}

SmaccToolActionClient::~SmaccToolActionClient()
{   
}
}

//PLUGINLIB_EXPORT_CLASS(smacc::SmaccToolActionClient, smacc::ISmaccComponent)
