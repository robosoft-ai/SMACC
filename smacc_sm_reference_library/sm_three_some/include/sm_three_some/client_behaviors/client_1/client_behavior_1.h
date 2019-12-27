
#pragma once
#include <smacc/smacc_client_behavior.h>
namespace sm_three_some
{
namespace client_1
{
class CbBehavior1 : public smacc::SmaccClientBehavior
{
public:
    typedef std_msgs::UInt16 TMessageType;

    //-------------------------------------------------------------------------------
    void onEntry()
    {
    }
};
} // namespace client_1
} // namespace sm_three_some