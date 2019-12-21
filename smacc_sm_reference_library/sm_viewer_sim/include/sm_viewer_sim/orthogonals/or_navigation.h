#include <smacc/smacc.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

namespace sm_viewer_sim
{
class OrNavigation : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<OrNavigation, smacc::ClMoveBaseZ>();
        client->name_ = "move_base";
        client->initialize();
    }
};
}