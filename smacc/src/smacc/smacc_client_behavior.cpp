#include <smacc/smacc_client_behavior.h>

namespace smacc
{
    void SmaccClientBehavior::onEntry()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior onEntry", this->getName().c_str());
    }

    void SmaccClientBehavior::onExit()
    {
        ROS_DEBUG("[%s] Default empty SmaccClientBehavior onExit", this->getName().c_str());
    }
} // namespace smacc