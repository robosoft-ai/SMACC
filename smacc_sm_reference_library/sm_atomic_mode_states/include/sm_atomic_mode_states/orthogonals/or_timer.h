#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace sm_atomic_mode_states
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<cl_ros_timer::ClRosTimer>(ros::Duration(1));
        client->initialize();
    }
};
} // namespace sm_atomic_mode_states
