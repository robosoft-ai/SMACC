#include <sm_atomic_services/sm_atomic_services.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_atomic_services");
    ros::NodeHandle nh;

    smacc::run<sm_atomic_services::SmAtomicServices>();
}
