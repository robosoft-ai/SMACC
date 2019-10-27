
#pragma once
#include <smacc/smacc_substate_behavior.h>

namespace hello_world_example
{
class SbBehavior2b : public smacc::SmaccSubStateBehavior
{
public:
typedef std_msgs::UInt16 TMessageType;
void onEntry()
{
}
};
} // namespace hello_world_example