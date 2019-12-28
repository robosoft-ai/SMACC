
#pragma once
#include <sm_three_some/clients/subscriber_client/cl_subscriber.h>

namespace sm_three_some
{
namespace subscriber_client
{
class CbDefaultSubscriberBehavior : public smacc::SmaccClientBehavior
{
public:
    typedef std_msgs::UInt16 TMessageType;

    //-------------------------------------------------------------------------------
    void onEntry()
    {
    }
};
} // namespace subscriber_client
} // namespace sm_three_some