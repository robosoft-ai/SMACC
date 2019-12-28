
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <sm_three_some/clients/updatable_publisher_client/cl_updatable_publisher.h>

namespace sm_three_some
{
namespace updatable_publisher_client
{
class CbMutedBehavior : public smacc::SmaccClientBehavior
{
public:
    void onEntry()
    {
    }
};
} // namespace client_2
} // namespace sm_three_some