#include <smacc_reelrbtx_plugin/reel.h>
#include <pluginlib/class_list_macros.h>


namespace smacc
{

SmaccReelActionClient::SmaccReelActionClient()
: Base("")
{

}

SmaccReelActionClient::SmaccReelActionClient(std::string action_server_namespace)
    : Base(action_server_namespace)
{
}

std::string SmaccReelActionClient::getName() const
{
    return "REEL ACTION CLIENT";
}

SmaccReelActionClient::~SmaccReelActionClient()
{   
}
}

PLUGINLIB_EXPORT_CLASS(smacc::SmaccReelActionClient, smacc::ISmaccActionClient)
