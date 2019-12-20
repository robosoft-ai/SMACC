#pragma once

#include <smacc/client_bases/smacc_service_client.h>
#include <std_srvs/SetBool.h>
namespace sm_dance_bot
{
class ClService3 : public smacc::SmaccServiceClient<std_srvs::SetBool>
{
public:
  ClService3()
  {
  }
};
} // namespace sm_dance_bot
