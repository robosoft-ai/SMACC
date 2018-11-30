#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <pluginlib/class_list_macros.h>

namespace smacc
{

SmaccMoveBaseActionClient::SmaccMoveBaseActionClient()
{
    //ROS_INFO("Smacc Move Base Action Client");
}

std::string SmaccMoveBaseActionClient::getName() const
{
    return "MOVE BASE ACTION CLIENT";
}

SmaccMoveBaseActionClient::~SmaccMoveBaseActionClient()
{   
}
}

PLUGINLIB_EXPORT_CLASS( smacc::SmaccMoveBaseActionClient, smacc::ISmaccComponent)
