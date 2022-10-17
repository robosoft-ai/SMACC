
#pragma once
#include <smacc/smacc_orthogonal.h>
#include <sm_subscriber/clients/cl_numbers_subscription/cl_numbers_subscription.h>

namespace sm_subscriber
{
class OrSubscriber : public smacc::Orthogonal<OrSubscriber>
{
public:
  virtual void onInitialize() override
  {
    auto subscriber_client = this->createClient<ClNumbersSubscription>("/numbers");
    subscriber_client ->initialize();
  }
};
}  // namespace sm_subscriber
