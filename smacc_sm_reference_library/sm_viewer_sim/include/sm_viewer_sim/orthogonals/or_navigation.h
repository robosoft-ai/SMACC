#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_action_client.h>

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