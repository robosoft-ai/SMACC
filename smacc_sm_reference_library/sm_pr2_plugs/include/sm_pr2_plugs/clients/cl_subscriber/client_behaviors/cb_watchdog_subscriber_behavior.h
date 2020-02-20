
#pragma once
#include <sm_pr2_plugs/clients/cl_subscriber/cl_subscriber.h>

namespace sm_pr2_plugs{
namespace cl_subscriber
{
class CbWatchdogSubscriberBehavior : public smacc::SmaccClientBehavior
{
public:
    typedef std_msgs::UInt16 TMessageType;

    void onEntry()
    {
    }
};
} // namespace cl_subscriber
} // namespace sm_pr2_plugs