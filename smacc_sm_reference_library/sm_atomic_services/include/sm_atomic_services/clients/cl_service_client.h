#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_service_client.h>

#include <std_srvs/Empty.h>

namespace sm_atomic_services
{
    class ClServiceClient : public smacc::client_bases::SmaccServiceClient<std_srvs::Empty>
    {
        public:
          ClServiceClient(const std::string& service_name) : smacc::client_bases::SmaccServiceClient<std_srvs::Empty>(service_name) {}
    };
}
