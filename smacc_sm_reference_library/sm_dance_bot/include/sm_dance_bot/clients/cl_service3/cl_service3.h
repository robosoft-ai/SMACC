#pragma once

#include <smacc/client_bases/smacc_service_client.h>
#include <std_srvs/SetBool.h>
namespace sm_dance_bot
{
namespace cl_service3
{
class ClService3 : public smacc::client_bases::SmaccServiceClient<std_srvs::SetBool>
{
public:
  ClService3()
  {
  }
};
} // namespace cl_service3
} // namespace sm_dance_bot
