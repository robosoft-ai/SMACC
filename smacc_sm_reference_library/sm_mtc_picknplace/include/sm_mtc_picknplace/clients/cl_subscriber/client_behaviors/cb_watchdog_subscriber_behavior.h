
#pragma once
#include <sm_mtc_picknplace/clients/cl_subscriber/cl_subscriber.h>

namespace sm_mtc_picknplace
{
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
} // namespace sm_mtc_picknplace