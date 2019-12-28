
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <sm_three_some/clients/updatable_publisher_client/cl_updatable_publisher.h>
namespace sm_three_some
{
namespace updatable_publisher_client
{
class CbDefaultPublishLoop : public smacc::SmaccClientBehavior,
                             public smacc::ISmaccUpdatable
{
public:
    void onEntry()
    {
    }

    virtual void update()
    {
    }
};
} // namespace updatable_publisher_client
} // namespace sm_three_some