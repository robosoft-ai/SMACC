#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace sm_atomic
{
class OrTimer : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<OrTimer, ros_timer_client::ClRosTimer>(ros::Duration(1));
        client->initialize();
    }
};
}