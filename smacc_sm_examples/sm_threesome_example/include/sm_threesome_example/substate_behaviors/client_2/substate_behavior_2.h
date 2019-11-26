
#pragma once
#include <smacc/smacc_substate_behavior.h>
namespace sm_threesome_example
{
class SbBehavior2 : public smacc::SmaccSubStateBehavior
{
public:
    typedef std_msgs::UInt16 TMessageType;
    void onEntry()
    {
    }
};
} // namespace sm_threesome_example