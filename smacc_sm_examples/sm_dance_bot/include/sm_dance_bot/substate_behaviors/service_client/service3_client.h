#pragma once

#include <smacc/interface_components/smacc_service_client.h>
#include <std_srvs/SetBool.h>
namespace sm_dancebot
{
class ServiceClient3 : public smacc::SmaccServiceClient<std_srvs::SetBool>
{
public:
  ServiceClient3()
  {
  }
};
} // namespace sm_dancebot
