#include <smacc/smacc.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_atomic_services
{
    class OrServices : public smacc::Orthogonal<OrServices>
    {
        virtual void onInitialize() override
        {
            auto service_server_client = this->createClient<ClServiceServer>("service");
            service_server_client->initialize();

            auto service_client = this->createClient<ClServiceClient>("service");
            service_client->initialize();
        }
    };
}
