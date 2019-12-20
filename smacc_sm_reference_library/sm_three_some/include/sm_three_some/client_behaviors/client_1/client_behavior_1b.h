
#pragma once
#include <smacc/smacc_client_behavior.h>
namespace sm_three_some
{
class CbBehavior1b : public smacc::SmaccClientBehavior
{
public:
    typedef std_msgs::UInt16 TMessageType;

    void onEntry()
    {
    }
};
} // namespace sm_three_some