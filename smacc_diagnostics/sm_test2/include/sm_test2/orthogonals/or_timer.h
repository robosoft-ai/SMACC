#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace sm_test2
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<cl_ros_timer_client::ClRosTimer>(ros::Duration(1));
        client->initialize();
    }
};
} // namespace sm_test2