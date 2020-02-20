
#pragma once
#include <sm_packML/clients/cl_subscriber/cl_subscriber.h>

namespace sm_packML
{
namespace cl_subscriber
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
} // namespace cl_subscriber
} // namespace sm_packML