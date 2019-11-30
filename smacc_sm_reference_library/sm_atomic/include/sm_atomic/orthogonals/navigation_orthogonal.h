#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_action_client.h>

namespace sm_atomic
{
class NavigationOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *client = this->createClient<smacc::SmaccMoveBaseActionClient>();
        client->name_ = "move_base";
        client->initialize();
    }
};
}