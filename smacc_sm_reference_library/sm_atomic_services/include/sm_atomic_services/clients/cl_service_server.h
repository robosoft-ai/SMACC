#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_service_server_client.h>

#include <std_srvs/Empty.h>

namespace sm_atomic_services
{
    class ClServiceServer : public smacc::client_bases::SmaccServiceServerClient<std_srvs::Empty>
    {
        public:
          ClServiceServer(const std::string& service_name) : smacc::client_bases::SmaccServiceServerClient<std_srvs::Empty>(service_name) {}
    };
}
