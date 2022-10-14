#include <smacc/smacc.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace sm_viewer_sim
{
using namespace cl_move_base_z;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        auto movebaseClient = this->createClient<ClMoveBaseZ>();
        movebaseClient->createComponent<cl_move_base_z::Pose>();

        movebaseClient->initialize();
    }
};
} // namespace sm_viewer_sim
