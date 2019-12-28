#include <smacc/smacc.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

namespace sm_viewer_sim
{
using namespace move_base_z_client;

class OrNavigation : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<OrNavigation, ClMoveBaseZ>();
        client->name_ = "move_base";
        client->initialize();
    }
};
} // namespace sm_viewer_sim